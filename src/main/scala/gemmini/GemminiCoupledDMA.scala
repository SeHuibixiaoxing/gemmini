package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.{BundleBridgeSink, IdRange, LazyModule}
import freechips.rocketchip.tile.{HasCoreParameters, LazyRoCC, LazyRoCCModuleImp, OpcodeSet}
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
  private val exportSharedSpadXlate =
    params.shared_scratchpad_config.enable && params.shared_scratchpad_config.share_xlate_with_coupled_dma

  val sharedSpadXlateSinkNodeOpt =
    if (exportSharedSpadXlate) Some(BundleBridgeSink[SharedSpadXlateConfig]()) else None
  sharedSpadXlateSinkNodeOpt.foreach { sink =>
    CoupledSharedSpadRegistry.connectXlateClient(params.gemmini_id, sink)
  }

  val dmaClientNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = s"Gemmini${params.gemmini_id}-CoupledDMA",
    sourceId = IdRange(0, 4),
    requestFifo = true
  )))))

  private val dmaXbar = TLXbar()
  dmaXbar := TLBuffer() := dmaClientNode

  val spmPtwOpt =
    if (exportSharedSpadXlate && params.shared_scratchpad_config.use_page_table_xlate) {
      Some(LazyModule(new SpmPageTableWalker(s"gemmini${params.gemmini_id}-cdma-spm-ptw")))
    } else {
      None
    }
  spmPtwOpt.foreach { ptw =>
    dmaXbar := TLBuffer() := ptw.node
  }

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
  extends LazyRoCCModuleImp(outer) with HasCoreParameters {
  private val xLenBits = io.cmd.bits.rs1.getWidth
  private val exportSharedSpadXlate =
    params.shared_scratchpad_config.enable && params.shared_scratchpad_config.share_xlate_with_coupled_dma
  private val spmTlbEntries = 8
  private val spmRefillIdxWidth = log2Ceil(spmTlbEntries max 2)

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

  val sharedSpadXlateCfg = Wire(new SharedSpadXlateConfig)
  sharedSpadXlateCfg := 0.U.asTypeOf(new SharedSpadXlateConfig)
  if (exportSharedSpadXlate) {
    require(outer.sharedSpadXlateSinkNodeOpt.get.in.nonEmpty,
      s"Gemmini${params.gemmini_id} coupled DMA missing shared-spad xlate sideband")
    sharedSpadXlateCfg := outer.sharedSpadXlateSinkNodeOpt.get.in.head._1
  }

  val spmPtwReqValid = WireInit(false.B)
  val spmPtwReqBits = Wire(new SpmPtwReq)
  spmPtwReqBits := 0.U.asTypeOf(new SpmPtwReq)
  val spmPtwReqReady = WireInit(false.B)
  val spmPtwResp = Wire(Valid(new SpmPtwResp))
  spmPtwResp.valid := false.B
  spmPtwResp.bits := 0.U.asTypeOf(new SpmPtwResp)

  outer.spmPtwOpt.foreach { ptw =>
    spmPtwReqReady := ptw.module.io.req.ready
    ptw.module.io.req.valid := spmPtwReqValid
    ptw.module.io.req.bits := spmPtwReqBits
    spmPtwResp.valid := ptw.module.io.resp.valid
    spmPtwResp.bits := ptw.module.io.resp.bits
  }

  val spmTlbValid = RegInit(VecInit(Seq.fill(spmTlbEntries)(false.B)))
  val spmTlbVpn = Reg(Vec(spmTlbEntries, UInt(vaddrBits.W)))
  val spmTlbPaddrBase = Reg(Vec(spmTlbEntries, UInt(paddrBits.W)))
  val spmRefillPtr = RegInit(0.U(spmRefillIdxWidth.W))
  val spmPtwPendingValid = RegInit(false.B)
  val spmPtwPendingVpn = RegInit(0.U(vaddrBits.W))
  val spmLastCacheEpoch = RegInit(0.U(8.W))
  val spmInvalidate = WireInit(false.B)

  when (spmPtwResp.valid && spmPtwPendingValid) {
    spmPtwPendingValid := false.B
    when (spmPtwResp.bits.accessFault) {
      assert(false.B, "GemminiCoupledDMA shared-spad PTW access fault")
    } .elsewhen (!spmPtwResp.bits.pte(0)) {
      assert(false.B, "GemminiCoupledDMA shared-spad PTW invalid PTE")
    } .otherwise {
      val refillIdx = spmRefillPtr
      val paddrBase = ((spmPtwResp.bits.pte(63, 1).asUInt << spmPtwResp.bits.pageShift)(paddrBits - 1, 0))
      spmTlbValid(refillIdx) := true.B
      spmTlbVpn(refillIdx) := spmPtwResp.bits.vpn
      spmTlbPaddrBase(refillIdx) := paddrBase
      if (spmTlbEntries > 1) {
        spmRefillPtr := Mux(spmRefillPtr === (spmTlbEntries - 1).U, 0.U, spmRefillPtr + 1.U)
      }
    }
  }

  if (exportSharedSpadXlate) {
    when (sharedSpadXlateCfg.cache_epoch =/= spmLastCacheEpoch) {
      spmInvalidate := true.B
      spmLastCacheEpoch := sharedSpadXlateCfg.cache_epoch
    }
  }

  when (spmInvalidate) {
    spmTlbValid.foreach(_ := false.B)
    spmPtwPendingValid := false.B
  }

  val resolvedSrcAddr = Wire(UInt(xLenBits.W))
  val resolvedDstAddr = Wire(UInt(xLenBits.W))
  resolvedSrcAddr := curSrc
  resolvedDstAddr := curDst

  val srcIssueBlocked = WireInit(false.B)
  val dstIssueBlocked = WireInit(false.B)
  val srcNeedPtwReq = WireInit(false.B)
  val dstNeedPtwReq = WireInit(false.B)
  val srcPtwReq = Wire(new SpmPtwReq)
  val dstPtwReq = Wire(new SpmPtwReq)
  srcPtwReq := 0.U.asTypeOf(new SpmPtwReq)
  dstPtwReq := 0.U.asTypeOf(new SpmPtwReq)

  if (exportSharedSpadXlate) {
    val srcVaddr = curSrc(vaddrBits - 1, 0)
    val srcRangeEnd = sharedSpadXlateCfg.range_base + sharedSpadXlateCfg.range_size
    val srcRangeHit = (sharedSpadXlateCfg.range_size =/= 0.U) &&
      srcVaddr >= sharedSpadXlateCfg.range_base && srcVaddr < srcRangeEnd
    val srcPageShift = sharedSpadXlateCfg.page_shift
    val srcOffset = srcVaddr - sharedSpadXlateCfg.range_base
    val srcVpn = srcOffset >> srcPageShift
    val srcVpnInBounds = srcVpn < sharedSpadXlateCfg.pte_count
    val srcPageOffset = srcOffset - (srcVpn << srcPageShift)
    val srcTlbHits = VecInit(spmTlbValid.zip(spmTlbVpn).map { case (valid, vpn) => valid && vpn === srcVpn })
    val srcTlbHit = srcTlbHits.asUInt.orR
    val srcTlbBase = Mux(srcTlbHit, Mux1H(srcTlbHits, spmTlbPaddrBase), 0.U(paddrBits.W))
    val srcPendingHit = spmPtwPendingValid && spmPtwPendingVpn === srcVpn
    val srcDirectHit = !sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && srcRangeHit && srcVpnInBounds
    val srcPassthroughHit = sharedSpadXlateCfg.use_ptw && !sharedSpadXlateCfg.enable && srcRangeHit
    val srcPtwHit = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && srcRangeHit && srcTlbHit
    val srcDirectPaddr = sharedSpadXlateCfg.shared_base + srcOffset
    val srcTranslatedPaddr = srcTlbBase + srcPageOffset
    val srcPaddr = Mux(srcPassthroughHit, curSrc, Mux(srcPtwHit, srcTranslatedPaddr, srcDirectPaddr))
    val srcRangeFault = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && srcRangeHit && !srcVpnInBounds
    val srcWaitForXlate = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && srcRangeHit && srcVpnInBounds && !srcTlbHit
    srcNeedPtwReq := srcWaitForXlate && !srcPendingHit && !spmPtwPendingValid
    srcIssueBlocked := srcRangeFault || srcWaitForXlate
    when (srcRangeFault) {
      assert(false.B, "GemminiCoupledDMA src shared-spad vaddr outside programmed PTE range")
    }
    when (srcDirectHit || srcPassthroughHit || srcPtwHit) {
      resolvedSrcAddr := srcPaddr
    }
    srcPtwReq.vpn := srcVpn
    srcPtwReq.vaddr := srcVaddr
    srcPtwReq.ptbr := sharedSpadXlateCfg.ptbr
    srcPtwReq.pageShift := srcPageShift

    val dstVaddr = curDst(vaddrBits - 1, 0)
    val dstRangeEnd = sharedSpadXlateCfg.range_base + sharedSpadXlateCfg.range_size
    val dstRangeHit = (sharedSpadXlateCfg.range_size =/= 0.U) &&
      dstVaddr >= sharedSpadXlateCfg.range_base && dstVaddr < dstRangeEnd
    val dstPageShift = sharedSpadXlateCfg.page_shift
    val dstOffset = dstVaddr - sharedSpadXlateCfg.range_base
    val dstVpn = dstOffset >> dstPageShift
    val dstVpnInBounds = dstVpn < sharedSpadXlateCfg.pte_count
    val dstPageOffset = dstOffset - (dstVpn << dstPageShift)
    val dstTlbHits = VecInit(spmTlbValid.zip(spmTlbVpn).map { case (valid, vpn) => valid && vpn === dstVpn })
    val dstTlbHit = dstTlbHits.asUInt.orR
    val dstTlbBase = Mux(dstTlbHit, Mux1H(dstTlbHits, spmTlbPaddrBase), 0.U(paddrBits.W))
    val dstPendingHit = spmPtwPendingValid && spmPtwPendingVpn === dstVpn
    val dstDirectHit = !sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && dstRangeHit && dstVpnInBounds
    val dstPassthroughHit = sharedSpadXlateCfg.use_ptw && !sharedSpadXlateCfg.enable && dstRangeHit
    val dstPtwHit = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && dstRangeHit && dstTlbHit
    val dstDirectPaddr = sharedSpadXlateCfg.shared_base + dstOffset
    val dstTranslatedPaddr = dstTlbBase + dstPageOffset
    val dstPaddr = Mux(dstPassthroughHit, curDst, Mux(dstPtwHit, dstTranslatedPaddr, dstDirectPaddr))
    val dstRangeFault = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && dstRangeHit && !dstVpnInBounds
    val dstWaitForXlate = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && dstRangeHit && dstVpnInBounds && !dstTlbHit
    dstNeedPtwReq := dstWaitForXlate && !dstPendingHit && !spmPtwPendingValid
    dstIssueBlocked := dstRangeFault || dstWaitForXlate
    when (dstRangeFault) {
      assert(false.B, "GemminiCoupledDMA dst shared-spad vaddr outside programmed PTE range")
    }
    when (dstDirectHit || dstPassthroughHit || dstPtwHit) {
      resolvedDstAddr := dstPaddr
    }
    dstPtwReq.vpn := dstVpn
    dstPtwReq.vaddr := dstVaddr
    dstPtwReq.ptbr := sharedSpadXlateCfg.ptbr
    dstPtwReq.pageShift := dstPageShift
  }

  when (state === sIssueGet && srcNeedPtwReq) {
    spmPtwReqValid := true.B
    spmPtwReqBits := srcPtwReq
  } .elsewhen (state === sIssuePut && dstNeedPtwReq) {
    spmPtwReqValid := true.B
    spmPtwReqBits := dstPtwReq
  }

  when (spmPtwReqValid && spmPtwReqReady) {
    spmPtwPendingValid := true.B
    spmPtwPendingVpn := spmPtwReqBits.vpn
  }

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

  val (_, getBits) = edge.Get(0.U, resolvedSrcAddr, nextLgSize)
  val (_, putCopyBits) = edge.Put(1.U, resolvedDstAddr, curLgSize, copyPutData)
  val (_, putFlagBits) = edge.Put(2.U, curCompletionAddr, 0.U, flagPutData)

  tl.a.valid := false.B
  tl.a.bits := getBits

  when (state === sIssueGet && !srcIssueBlocked && !srcNeedPtwReq) {
    tl.a.valid := true.B
    tl.a.bits := getBits
  } .elsewhen (state === sIssuePut && !dstIssueBlocked && !dstNeedPtwReq) {
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
  val acceptSfence = io.cmd.fire && isSfence

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

  when (acceptSfence) {
    spmInvalidate := true.B
  }

  io.busy := dmaBusy || respValid
}
