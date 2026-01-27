
package gemmini

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.util.DecoupledHelper
import freechips.rocketchip.rocket.{TLBConfig}
import freechips.rocketchip.diplomacy._
import roccaccutils._
import roccaccutils.logger._

// --- DMA Implementation ---

class GemminiDirectDMA(opcodes: OpcodeSet)(implicit p: Parameters)
  extends MemStreamerAccel(opcodes) {
  
  override lazy val tlbConfig = TLBConfig(nSets = 4, nWays = 4, nSectors = 1, nSuperpageEntries = 4)
  override lazy val xbarBetweenMem = false
  override lazy val logger = new Logger {
    override def logInfoImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = printf
    override def logCriticalImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = printf
  }

  lazy val module = new GemminiDirectDMAImp(this)
}

class GemminiDirectDMAImp(outer: GemminiDirectDMA)(implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) with freechips.rocketchip.rocket.constants.MemoryOpConstants {
  
  // -------------------------------------------------------------
  // Flattened MemStreamerAccelImp logic to override cmd_router
  // -------------------------------------------------------------

  implicit val hp: L2MemHelperParams = outer.hp

  io.mem.req.valid := false.B
  io.mem.s1_kill := false.B
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := true.B
  io.interrupt := false.B
  io.busy := false.B

  lazy val queueDepth = 16

  // Instantiate the Command Router provided by rocc-acc-utils
  // It decodes custom instructions into src_info and dest_info
  class CmdRouterImpl(implicit val p: Parameters) extends Module with StreamingCommandRouter {
    lazy val io = IO(new MemStreamerCmdBundle())
    lazy val cmd_queue_depth = queueDepth

    io.rocc_in.ready := streaming_fire
  }

  val cmd_router = Module(new CmdRouterImpl()(outer.p))

  // Instantiate the MemStreamer which connects MemLoader to MemWriter
  // This is the "Copy Engine"
  class StreamerImpl(val logger: Logger)(implicit val p: Parameters, val hp: L2MemHelperParams) extends Module with MemStreamer {
    lazy val io = IO(new MemStreamerBundle)

    // Connect loader output to writer input
    // Effectively a FIFO: MemLoader -> load_data_queue -> MemWriter
    // MemStreamer trait already defines load_data_queue and connects it to io.mem_stream (loader)

    // We just need to connect the output of load_data_queue to the store_data_queue defined in the trait
    val q = load_data_queue

    store_data_queue.io.enq.bits.chunk_data := q.io.deq.bits.chunk_data
    store_data_queue.io.enq.bits.chunk_size_bytes := q.io.deq.bits.chunk_size_bytes
    store_data_queue.io.enq.bits.is_final_chunk := q.io.deq.bits.is_final_chunk

    store_data_queue.io.enq.valid := q.io.deq.valid
    q.io.deq.ready := store_data_queue.io.enq.ready
  }

  val streamer = Module(new StreamerImpl(outer.logger)(outer.p, outer.hp))

  // --------------------------
  // Wiring from MemStreamerAccelImp
  // --------------------------
  
  val memloader = Module(new MemLoader(memLoaderQueDepth=queueDepth, logger=outer.logger))
  outer.l2_memloader.module.io.userif <> memloader.io.l2helperUser
  memloader.io.src_info <> cmd_router.io.src_info

  val memwriter = Module(new MemWriter32(cmd_que_depth=queueDepth, logger=outer.logger))
  outer.l2_memwriter.module.io.userif <> memwriter.io.l2io

  outer.l2_memloader.module.io.sfence <> cmd_router.io.sfence_out
  outer.l2_memloader.module.io.status.valid := cmd_router.io.dmem_status_out.valid
  outer.l2_memloader.module.io.status.bits := cmd_router.io.dmem_status_out.bits.status
  io.ptw(0) <> outer.l2_memloader.module.io.ptw

  outer.l2_memwriter.module.io.sfence <> cmd_router.io.sfence_out
  outer.l2_memwriter.module.io.status.valid := cmd_router.io.dmem_status_out.valid
  outer.l2_memwriter.module.io.status.bits := cmd_router.io.dmem_status_out.bits.status
  io.ptw(1) <> outer.l2_memwriter.module.io.ptw

  cmd_router.io.rocc_in <> io.cmd
  io.resp <> cmd_router.io.rocc_out

  streamer.io.mem_stream <> memloader.io.consumer
  memwriter.io.memwrites_in <> streamer.io.memwrites_in
  memwriter.io.decompress_dest_info <> cmd_router.io.dest_info
  cmd_router.io.bufs_completed := memwriter.io.bufs_completed
  cmd_router.io.no_writes_inflight := memwriter.io.no_writes_inflight
}


// --- Config Fragment ---

class WithGemminiDirectDMA extends org.chipsalliance.cde.config.Config((site, here, up) => {
  case freechips.rocketchip.tile.BuildRoCC => up(freechips.rocketchip.tile.BuildRoCC) ++ Seq(
    (p: Parameters) => {
      val dma = LazyModule(new GemminiDirectDMA(OpcodeSet.custom2)(p)) 
      dma
    }
  )
})
