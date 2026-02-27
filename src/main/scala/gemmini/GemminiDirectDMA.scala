
package gemmini

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.util.DecoupledHelper
import freechips.rocketchip.rocket.{TLBConfig}
import freechips.rocketchip.diplomacy._
import midas.targetutils.{SynthesizePrintf}
import roccaccutils._
import roccaccutils.logger._

// Logger for GemminiDirectDMA
object GemminiDirectDMALogger extends Logger {
  override def logInfoImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = {
    SynthesizePrintf(printf)
  }
  
  override def logCriticalImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = {
    SynthesizePrintf(printf)
  }
}

// --- DMA Implementation ---

class GemminiDirectDMA(opcodes: OpcodeSet)(implicit p: Parameters)
  extends { override val hp: L2MemHelperParams = L2MemHelperParams(32 * 8) }
  with MemStreamerAccel(opcodes) {
  
  override lazy val tlbConfig = TLBConfig(nSets = 4, nWays = 4, nSectors = 1, nSuperpageEntries = 4)
  override lazy val xbarBetweenMem = false
  override lazy val logger = GemminiDirectDMALogger

  lazy val module = new GemminiDirectDMAImp(this)
}

class GemminiDirectDMAImp(outer: GemminiDirectDMA)(implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) with freechips.rocketchip.rocket.constants.MemoryOpConstants {
  
  // -------------------------------------------------------------
  // Flattened MemStreamerAccelImp logic to override cmd_router
  // -------------------------------------------------------------

  implicit val hp: L2MemHelperParams = outer.hp

  // Disable memory interface (using TileLink instead)
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
  class StreamerImpl(val logger: Logger)(implicit val p: Parameters, val hp: L2MemHelperParams)
    extends Module with HasL2MemHelperParams {
    lazy val io = IO(new MemStreamerBundle)

    val load_data_queue = Module(new Queue(new LiteralChunk, 5))
    dontTouch(load_data_queue.io.count)

    load_data_queue.io.enq.bits.chunk_data := io.mem_stream.output_data
    load_data_queue.io.enq.bits.chunk_size_bytes := io.mem_stream.available_output_bytes
    load_data_queue.io.enq.bits.is_final_chunk := io.mem_stream.output_last_chunk
    val fire_read = io.mem_stream.output_valid && load_data_queue.io.enq.ready
    load_data_queue.io.enq.valid := fire_read

    // Break combinational loop on output_ready/user_consumed_bytes.
    io.mem_stream.output_ready := load_data_queue.io.enq.ready
    io.mem_stream.user_consumed_bytes := io.mem_stream.available_output_bytes

    val store_data_queue = Module(new Queue(new LiteralChunk, 5))
    dontTouch(store_data_queue.io.count)

    store_data_queue.io.enq.bits.chunk_data := load_data_queue.io.deq.bits.chunk_data
    store_data_queue.io.enq.bits.chunk_size_bytes := load_data_queue.io.deq.bits.chunk_size_bytes
    store_data_queue.io.enq.bits.is_final_chunk := load_data_queue.io.deq.bits.is_final_chunk
    store_data_queue.io.enq.valid := load_data_queue.io.deq.valid
    load_data_queue.io.deq.ready := store_data_queue.io.enq.ready

    val sdq_chunk_size = store_data_queue.io.deq.bits.chunk_size_bytes
    val sdq_chunk_data = store_data_queue.io.deq.bits.chunk_data
    val sdq_chunk_data_vec = VecInit(Seq.fill(BUS_SZ_BYTES)(0.U(8.W)))
    for (i <- 0 to (BUS_SZ_BYTES - 1)) {
      sdq_chunk_data_vec(sdq_chunk_size - 1.U - i.U) := sdq_chunk_data((8*(i+1))-1, 8*i)
    }
    io.memwrites_in.bits.data := sdq_chunk_data_vec.asUInt
    io.memwrites_in.bits.validbytes := sdq_chunk_size
    io.memwrites_in.bits.end_of_message := store_data_queue.io.deq.bits.is_final_chunk
    io.memwrites_in.valid := store_data_queue.io.deq.valid
    store_data_queue.io.deq.ready := io.memwrites_in.ready

  }

  val streamer = Module(new StreamerImpl(outer.logger)(outer.p, outer.hp))


  val memloader = Module(new MemLoader(memLoaderQueDepth=64, logger=outer.logger))
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
  cmd_router.io.bus_write_bytes := memwriter.io.bus_write_bytes
}


// --- Config Fragment ---

class WithGemminiDirectDMA extends org.chipsalliance.cde.config.Config((site, here, up) => {
  case freechips.rocketchip.tile.BuildRoCC => up(freechips.rocketchip.tile.BuildRoCC) ++ Seq(
    (p: Parameters) => LazyModule(new GemminiDirectDMA(OpcodeSet.custom2)(p))
  )
})
