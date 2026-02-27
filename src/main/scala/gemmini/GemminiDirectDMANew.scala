package gemmini

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util.DecoupledHelper
import freechips.rocketchip.rocket.TLBConfig
import freechips.rocketchip.diplomacy._
import midas.targetutils.SynthesizePrintf
import roccaccutils._
import roccaccutils.logger._

object GemminiDirectDMANewLogger extends Logger {
  override def logInfoImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = {
    SynthesizePrintf(printf)
  }

  override def logCriticalImplPrintWrapper(printf: chisel3.printf.Printf)(implicit p: Parameters): chisel3.printf.Printf = {
    SynthesizePrintf(printf)
  }
}

trait ParallelSafeStreamingCommandRouter extends Module {
  val io: MemStreamerCmdBundle
  val cmd_queue_depth: Int
  implicit val p: Parameters

  val FUNCT_SFENCE           = 0.U
  val FUNCT_SRC_INFO         = 1.U
  val FUNCT_DEST_INFO        = 2.U
  val FUNCT_CHECK_COMPLETION = 3.U
  val FUNCT_READ_MONITOR     = 4.U

  val DMA_MON_VALID               = 0.U(64.W)
  val DMA_MON_SRC_CMDS            = 1.U(64.W)
  val DMA_MON_DST_CMDS            = 2.U(64.W)
  val DMA_MON_REQ_COPY_BYTES      = 3.U(64.W)
  val DMA_MON_CYCLES              = 4.U(64.W)
  val DMA_MON_EFFECTIVE_BYTES     = 5.U(64.W)
  val DMA_MON_EFF_BW_X1000_BPC    = 6.U(64.W)

  val cur_funct = io.rocc_in.bits.inst.funct
  val cur_rs1 = io.rocc_in.bits.rs1
  val cur_rs2 = io.rocc_in.bits.rs2

  val sfence_fire = DecoupledHelper(
    io.rocc_in.valid,
    cur_funct === FUNCT_SFENCE
  )
  io.sfence_out := sfence_fire.fire()

  io.dmem_status_out.bits <> io.rocc_in.bits
  io.dmem_status_out.valid <> io.rocc_in.fire

  val src_info_queue = Module(new Queue(new StreamInfo, cmd_queue_depth))
  src_info_queue.io.enq.bits.ip := cur_rs1
  src_info_queue.io.enq.bits.isize := cur_rs2
  val src_info_fire = DecoupledHelper(
    io.rocc_in.valid,
    cur_funct === FUNCT_SRC_INFO,
    src_info_queue.io.enq.ready
  )
  src_info_queue.io.enq.valid := src_info_fire.fire(src_info_queue.io.enq.ready)
  io.src_info <> src_info_queue.io.deq

  val dest_info_queue = Module(new Queue(new DstInfo, cmd_queue_depth))
  dest_info_queue.io.enq.bits.op := cur_rs1
  dest_info_queue.io.enq.bits.cmpflag := cur_rs2
  val dest_info_fire = DecoupledHelper(
    io.rocc_in.valid,
    cur_funct === FUNCT_DEST_INFO,
    dest_info_queue.io.enq.ready
  )
  dest_info_queue.io.enq.valid := dest_info_fire.fire(dest_info_queue.io.enq.ready)
  io.dest_info <> dest_info_queue.io.deq

  val track_dispatched_src_infos = RegInit(0.U(64.W))
  val bufs_completed_base = RegInit(0.U(64.W))

  val monitor_active = RegInit(false.B)
  val monitor_cycle = RegInit(0.U(64.W))
  val monitor_start_cycle = RegInit(0.U(64.W))
  val monitor_src_cmds = RegInit(0.U(64.W))
  val monitor_dst_cmds = RegInit(0.U(64.W))
  val monitor_req_copy_bytes = RegInit(0.U(64.W))
  val monitor_bus_bytes_base = RegInit(0.U(64.W))

  val monitor_last_valid = RegInit(false.B)
  val monitor_last_src_cmds = RegInit(0.U(64.W))
  val monitor_last_dst_cmds = RegInit(0.U(64.W))
  val monitor_last_req_copy_bytes = RegInit(0.U(64.W))
  val monitor_last_cycles = RegInit(0.U(64.W))
  val monitor_last_effective_bytes = RegInit(0.U(64.W))

  monitor_cycle := monitor_cycle + 1.U

  when(io.rocc_in.fire && cur_funct === FUNCT_SRC_INFO) {
    when(!monitor_active) {
      monitor_active := true.B
      monitor_start_cycle := monitor_cycle
      monitor_src_cmds := 0.U
      monitor_dst_cmds := 0.U
      monitor_req_copy_bytes := 0.U
      monitor_bus_bytes_base := io.bus_write_bytes
      monitor_last_valid := false.B
    }

    monitor_src_cmds := monitor_src_cmds + 1.U
    monitor_req_copy_bytes := monitor_req_copy_bytes + cur_rs2

    when(track_dispatched_src_infos === 0.U) {
      bufs_completed_base := io.bufs_completed
    }
    track_dispatched_src_infos := track_dispatched_src_infos + 1.U
  }

  when(io.rocc_in.fire && cur_funct === FUNCT_DEST_INFO) {
    monitor_dst_cmds := monitor_dst_cmds + 1.U
  }

  val completed_since_base = io.bufs_completed - bufs_completed_base
  val all_dispatched_completed = completed_since_base >= track_dispatched_src_infos

  val check_completion_fire = DecoupledHelper(
    io.rocc_in.valid,
    cur_funct === FUNCT_CHECK_COMPLETION,
    io.no_writes_inflight,
    track_dispatched_src_infos =/= 0.U,
    all_dispatched_completed,
    io.rocc_out.ready
  )

  when(io.rocc_in.fire && cur_funct === FUNCT_CHECK_COMPLETION) {
    val monitor_cycles_this = monitor_cycle - monitor_start_cycle
    val monitor_bus_bytes_this = io.bus_write_bytes - monitor_bus_bytes_base
    monitor_last_valid := monitor_active
    monitor_last_src_cmds := monitor_src_cmds
    monitor_last_dst_cmds := monitor_dst_cmds
    monitor_last_req_copy_bytes := monitor_req_copy_bytes
    monitor_last_cycles := Mux(monitor_active && monitor_cycles_this === 0.U, 1.U, monitor_cycles_this)
    monitor_last_effective_bytes := monitor_bus_bytes_this
    monitor_active := false.B

    track_dispatched_src_infos := 0.U
    bufs_completed_base := io.bufs_completed
  }

  val monitor_data = Wire(UInt(64.W))
  monitor_data := 0.U
  when (cur_rs1 === DMA_MON_VALID) {
    monitor_data := monitor_last_valid
  } .elsewhen (cur_rs1 === DMA_MON_SRC_CMDS) {
    monitor_data := monitor_last_src_cmds
  } .elsewhen (cur_rs1 === DMA_MON_DST_CMDS) {
    monitor_data := monitor_last_dst_cmds
  } .elsewhen (cur_rs1 === DMA_MON_REQ_COPY_BYTES) {
    monitor_data := monitor_last_req_copy_bytes
  } .elsewhen (cur_rs1 === DMA_MON_CYCLES) {
    monitor_data := monitor_last_cycles
  } .elsewhen (cur_rs1 === DMA_MON_EFFECTIVE_BYTES) {
    monitor_data := monitor_last_effective_bytes
  } .elsewhen (cur_rs1 === DMA_MON_EFF_BW_X1000_BPC) {
    monitor_data := 0.U
  }

  val read_monitor_fire = DecoupledHelper(
    io.rocc_in.valid,
    cur_funct === FUNCT_READ_MONITOR,
    io.rocc_out.ready
  )

  io.rocc_out.valid := check_completion_fire.fire(io.rocc_out.ready) ||
    read_monitor_fire.fire(io.rocc_out.ready)
  io.rocc_out.bits.data := Mux(read_monitor_fire.fire(io.rocc_out.ready), monitor_data, track_dispatched_src_infos)
  io.rocc_out.bits.rd := io.rocc_in.bits.inst.rd

  val streaming_fire = sfence_fire.fire(io.rocc_in.valid) ||
    src_info_fire.fire(io.rocc_in.valid) ||
    dest_info_fire.fire(io.rocc_in.valid) ||
    check_completion_fire.fire(io.rocc_in.valid) ||
    read_monitor_fire.fire(io.rocc_in.valid)
}

class GemminiDirectDMANew(opcodes: OpcodeSet)(implicit p: Parameters)
  extends MemStreamerAccel(opcodes) {

  override lazy val tlbConfig = TLBConfig(nSets = 4, nWays = 4, nSectors = 1, nSuperpageEntries = 4)
  override lazy val xbarBetweenMem = false
  override lazy val logger = GemminiDirectDMANewLogger

  lazy val module = new GemminiDirectDMANewImp(this)
}

class GemminiDirectDMANewImp(outer: GemminiDirectDMANew)(implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) with freechips.rocketchip.rocket.constants.MemoryOpConstants {

  implicit val hp: L2MemHelperParams = outer.hp

  io.mem.req.valid := false.B
  io.mem.s1_kill := false.B
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := true.B
  io.interrupt := false.B
  io.busy := false.B

  lazy val queueDepth = 16

  class CmdRouterImpl(implicit val p: Parameters) extends Module with ParallelSafeStreamingCommandRouter {
    lazy val io = IO(new MemStreamerCmdBundle())
    lazy val cmd_queue_depth = queueDepth

    io.rocc_in.ready := streaming_fire
  }

  val cmd_router = Module(new CmdRouterImpl()(outer.p))

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

  val memloader = Module(new MemLoader(memLoaderQueDepth = 64, logger = outer.logger))
  outer.l2_memloader.module.io.userif <> memloader.io.l2helperUser
  memloader.io.src_info <> cmd_router.io.src_info

  val memwriter = Module(new MemWriter32(cmd_que_depth = queueDepth, logger = outer.logger))
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
