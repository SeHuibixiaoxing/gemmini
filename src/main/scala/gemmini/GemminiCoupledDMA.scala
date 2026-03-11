package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.{IdRange, LazyModule}
import freechips.rocketchip.tile.{LazyRoCC, LazyRoCCModuleImp, OpcodeSet}
import freechips.rocketchip.tilelink._

case class CoupledDMAParams(
  gemmini_id: Int,
  shared_scratchpad_config: SharedScratchpadConfig,
  copy_queue_depth: Int = 8,
  enable_monitor_counters: Boolean = true
)

class GemminiCoupledDMA(opcodes: OpcodeSet, params: CoupledDMAParams)(implicit p: Parameters)
  extends LazyRoCC(opcodes = opcodes, nPTWPorts = 0) {

  private val localBankAddrSets = params.shared_scratchpad_config.local_bank_addr_sets(params.gemmini_id)

  val dmaClientNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = s"Gemmini${params.gemmini_id}-CoupledDMA",
    sourceId = IdRange(0, 4),
    requestFifo = true
  )))))

  private val dmaXbar = TLXbar()
  dmaXbar := TLBuffer() := dmaClientNode

  // Route local shared-spad traffic directly into the coupled Gemmini ingress.
  private val localClientNode = TLBuffer() := dmaXbar
  CoupledSharedSpadRegistry.connectClient(params.gemmini_id, localClientNode)

  private val masterNode = TLIdentityNode()
  masterNode := TLFilter(TLFilter.mSubtract(localBankAddrSets)) := dmaXbar

  override val tlNode = masterNode
  override val atlNode = TLIdentityNode()

  lazy val module = new GemminiCoupledDMAImp(this, params)
}

class GemminiCoupledDMAImp(outer: GemminiCoupledDMA, params: CoupledDMAParams)(implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) {
  private val xLenBits = io.cmd.bits.rs1.getWidth

  private val FUNCT_SFENCE = 0.U(7.W)
  private val FUNCT_SRC_INFO = 1.U(7.W)
  private val FUNCT_DEST_INFO = 2.U(7.W)
  private val FUNCT_CHECK_COMPLETION = 3.U(7.W)
  private val FUNCT_READ_MONITOR = 4.U(7.W)

  private val MON_VALID = 0.U(64.W)
  private val MON_SRC_CMDS = 1.U(64.W)
  private val MON_DST_CMDS = 2.U(64.W)
  private val MON_REQ_COPY_BYTES = 3.U(64.W)
  private val MON_CYCLES = 4.U(64.W)
  private val MON_EFFECTIVE_BYTES = 5.U(64.W)
  private val MON_EFF_BW_X1000_BPC = 6.U(64.W)

  io.mem.req.valid := false.B
  io.mem.s1_kill := false.B
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := true.B
  io.interrupt := false.B

  val (tl, edge) = outer.dmaClientNode.out(0)
  val dataBits = tl.params.dataBits
  val beatBytes = dataBits / 8
  val beatOffBits = log2Ceil(beatBytes)
  val wideBytes = beatBytes
  val wideLgSize = log2Ceil(beatBytes).U

  class CopyReq extends Bundle {
    val src = UInt(xLenBits.W)
    val dst = UInt(xLenBits.W)
    val len = UInt(xLenBits.W)
    val completion = UInt(xLenBits.W)
  }

  val copyReqQ = Module(new Queue(new CopyReq, params.copy_queue_depth))
  copyReqQ.io.enq.valid := false.B
  copyReqQ.io.enq.bits := DontCare
  copyReqQ.io.deq.ready := false.B

  val dstAddrReg = RegInit(0.U(xLenBits.W))
  val completionAddrReg = RegInit(0.U(xLenBits.W))

  val srcCmdCount = RegInit(0.U(64.W))
  val dstCmdCount = RegInit(0.U(64.W))
  val reqCopyBytes = RegInit(0.U(64.W))
  val effectiveBytes = RegInit(0.U(64.W))
  val cycleCount = RegInit(0.U(64.W))
  cycleCount := cycleCount + 1.U

  val sIdle :: sIssueGet :: sWaitGet :: sIssuePut :: sWaitPut :: sIssueFlag :: sWaitFlag :: Nil = Enum(7)
  val state = RegInit(sIdle)

  val curSrc = Reg(UInt(xLenBits.W))
  val curDst = Reg(UInt(xLenBits.W))
  val curRemaining = Reg(UInt(xLenBits.W))
  val curCompletionAddr = Reg(UInt(xLenBits.W))
  val curReadData = Reg(UInt(dataBits.W))
  val curXferBytes = Reg(UInt(log2Ceil(wideBytes + 1).W))
  val curLgSize = Reg(UInt(4.W))

  val srcBeatAligned = if (beatOffBits > 0) curSrc(beatOffBits - 1, 0) === 0.U else true.B
  val dstBeatAligned = if (beatOffBits > 0) curDst(beatOffBits - 1, 0) === 0.U else true.B
  val canWide = srcBeatAligned && dstBeatAligned && curRemaining >= wideBytes.U
  val nextXferBytes = Mux(canWide, wideBytes.U, 1.U)
  val nextLgSize = Mux(canWide, wideLgSize, 0.U)

  val srcByteShift = Cat(curSrc(beatOffBits - 1, 0), 0.U(3.W))
  val dstByteShift = Cat(curDst(beatOffBits - 1, 0), 0.U(3.W))
  val movedByte = (curReadData >> srcByteShift)(7, 0)
  val bytePutData = (movedByte.asUInt << dstByteShift)(dataBits - 1, 0)
  val widePutData = curReadData
  val copyPutData = Mux(curXferBytes === wideBytes.U, widePutData, bytePutData)

  val flagByteShift = Cat(curCompletionAddr(beatOffBits - 1, 0), 0.U(3.W))
  val flagPutData = (1.U(dataBits.W) << flagByteShift)(dataBits - 1, 0)

  val (_, getBits) = edge.Get(0.U, curSrc, nextLgSize)
  val (_, putCopyBits) = edge.Put(1.U, curDst, curLgSize, copyPutData)
  val (_, putFlagBits) = edge.Put(2.U, curCompletionAddr, 0.U, flagPutData)

  tl.a.valid := false.B
  tl.a.bits := getBits

  when (state === sIssueGet) {
    tl.a.valid := true.B
    tl.a.bits := getBits
  } .elsewhen (state === sIssuePut) {
    tl.a.valid := true.B
    tl.a.bits := putCopyBits
  } .elsewhen (state === sIssueFlag) {
    tl.a.valid := true.B
    tl.a.bits := putFlagBits
  }

  tl.d.ready := state === sWaitGet || state === sWaitPut || state === sWaitFlag

  when (state === sIdle && copyReqQ.io.deq.valid) {
    copyReqQ.io.deq.ready := true.B
    curSrc := copyReqQ.io.deq.bits.src
    curDst := copyReqQ.io.deq.bits.dst
    curRemaining := copyReqQ.io.deq.bits.len
    curCompletionAddr := copyReqQ.io.deq.bits.completion
    state := Mux(copyReqQ.io.deq.bits.len === 0.U, sIssueFlag, sIssueGet)
  }

  when (state === sIssueGet && tl.a.fire) {
    curXferBytes := nextXferBytes
    curLgSize := nextLgSize
    state := sWaitGet
  }

  when (state === sWaitGet && tl.d.fire) {
    curReadData := tl.d.bits.data
    state := sIssuePut
  }

  when (state === sIssuePut && tl.a.fire) {
    state := sWaitPut
  }

  when (state === sWaitPut && tl.d.fire) {
    val nextRemaining = curRemaining - curXferBytes
    curSrc := curSrc + curXferBytes
    curDst := curDst + curXferBytes
    curRemaining := nextRemaining
    effectiveBytes := effectiveBytes + curXferBytes
    state := Mux(nextRemaining === 0.U, sIssueFlag, sIssueGet)
  }

  when (state === sIssueFlag && tl.a.fire) {
    state := sWaitFlag
  }

  when (state === sWaitFlag && tl.d.fire) {
    state := sIdle
  }

  val dmaBusy = state =/= sIdle || copyReqQ.io.deq.valid

  val respValid = RegInit(false.B)
  val respData = RegInit(0.U(xLenBits.W))
  val respRd = RegInit(0.U(5.W))

  io.resp.valid := respValid
  io.resp.bits.rd := respRd
  io.resp.bits.data := respData
  when (io.resp.fire) {
    respValid := false.B
  }

  val monitorData = Wire(UInt(64.W))
  monitorData := 0.U
  when (io.cmd.bits.rs1 === MON_VALID) {
    monitorData := 1.U
  } .elsewhen (io.cmd.bits.rs1 === MON_SRC_CMDS) {
    monitorData := srcCmdCount
  } .elsewhen (io.cmd.bits.rs1 === MON_DST_CMDS) {
    monitorData := dstCmdCount
  } .elsewhen (io.cmd.bits.rs1 === MON_REQ_COPY_BYTES) {
    monitorData := reqCopyBytes
  } .elsewhen (io.cmd.bits.rs1 === MON_CYCLES) {
    monitorData := cycleCount
  } .elsewhen (io.cmd.bits.rs1 === MON_EFFECTIVE_BYTES) {
    monitorData := effectiveBytes
  } .elsewhen (io.cmd.bits.rs1 === MON_EFF_BW_X1000_BPC) {
    monitorData := 0.U
  }

  val funct = io.cmd.bits.inst.funct
  val canAcceptRespCmd = !respValid
  val canCompleteFence = !dmaBusy

  val isDest = funct === FUNCT_DEST_INFO
  val isSrc = funct === FUNCT_SRC_INFO
  val isFence = funct === FUNCT_CHECK_COMPLETION
  val isMonitor = funct === FUNCT_READ_MONITOR
  val isSfence = funct === FUNCT_SFENCE

  val readyDest = isDest
  val readySrc = isSrc && copyReqQ.io.enq.ready
  val readyFence = isFence && canAcceptRespCmd && canCompleteFence
  val readyMonitor = isMonitor && canAcceptRespCmd
  val readySfence = isSfence

  io.cmd.ready := readyDest || readySrc || readyFence || readyMonitor || readySfence

  val acceptDest = io.cmd.fire && isDest
  val acceptSrc = io.cmd.fire && isSrc
  val acceptFence = io.cmd.fire && isFence
  val acceptMonitor = io.cmd.fire && isMonitor

  when (acceptDest) {
    dstAddrReg := io.cmd.bits.rs1
    completionAddrReg := io.cmd.bits.rs2
    dstCmdCount := dstCmdCount + 1.U
  }

  when (acceptSrc) {
    copyReqQ.io.enq.valid := true.B
    copyReqQ.io.enq.bits.src := io.cmd.bits.rs1
    copyReqQ.io.enq.bits.dst := dstAddrReg
    copyReqQ.io.enq.bits.len := io.cmd.bits.rs2
    copyReqQ.io.enq.bits.completion := completionAddrReg
    srcCmdCount := srcCmdCount + 1.U
    reqCopyBytes := reqCopyBytes + io.cmd.bits.rs2
  }

  when (acceptFence) {
    respValid := true.B
    respRd := io.cmd.bits.inst.rd
    respData := 0.U
  }

  when (acceptMonitor) {
    respValid := true.B
    respRd := io.cmd.bits.inst.rd
    respData := monitorData
  }

  io.busy := dmaBusy || respValid
}
