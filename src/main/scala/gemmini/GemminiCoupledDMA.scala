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

  val Seq(sIdle, sIssueGet0, sWaitGet0, sIssueGet1, sWaitGet1,
    sIssuePut, sWaitPut, sIssueFlag, sWaitFlag) = Enum(9)
  val state = RegInit(sIdle)

  val curSrc = Reg(UInt(xLenBits.W))
  val curDst = Reg(UInt(xLenBits.W))
  val curRemaining = Reg(UInt(xLenBits.W))
  val curCompletionAddr = Reg(UInt(xLenBits.W))
  val curReadDataLo = Reg(UInt(dataBits.W))
  val curReadDataHi = Reg(UInt(dataBits.W))
  val curXferBytes = Reg(UInt(log2Ceil(wideBytes + 1).W))
  val cachedReadValid = RegInit(false.B)
  val cachedReadBase = RegInit(0.U(xLenBits.W))
  val cachedReadData = RegInit(0.U(dataBits.W))

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

  class ResolvedCopyAddr extends Bundle {
    val paddr = UInt(xLenBits.W)
    val issueBlocked = Bool()
    val needPtwReq = Bool()
    val ptwReq = new SpmPtwReq
  }

  def resolveCopyAddr(rawAddr: UInt, errorLabel: String): ResolvedCopyAddr = {
    val resolved = Wire(new ResolvedCopyAddr)
    resolved := 0.U.asTypeOf(new ResolvedCopyAddr)
    resolved.paddr := rawAddr

    if (exportSharedSpadXlate) {
      val vaddr = rawAddr(vaddrBits - 1, 0)
      val rangeEnd = sharedSpadXlateCfg.range_base + sharedSpadXlateCfg.range_size
      val rangeHit = (sharedSpadXlateCfg.range_size =/= 0.U) &&
        vaddr >= sharedSpadXlateCfg.range_base && vaddr < rangeEnd
      val pageShift = sharedSpadXlateCfg.page_shift
      val offset = vaddr - sharedSpadXlateCfg.range_base
      val vpn = offset >> pageShift
      val vpnInBounds = vpn < sharedSpadXlateCfg.pte_count
      val pageOffset = offset - (vpn << pageShift)
      val tlbHits = VecInit(spmTlbValid.zip(spmTlbVpn).map { case (valid, tlbVpn) => valid && tlbVpn === vpn })
      val tlbHit = tlbHits.asUInt.orR
      val tlbBase = Mux(tlbHit, Mux1H(tlbHits, spmTlbPaddrBase), 0.U(paddrBits.W))
      val pendingHit = spmPtwPendingValid && spmPtwPendingVpn === vpn
      val directHit = !sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && rangeHit && vpnInBounds
      val passthroughHit = sharedSpadXlateCfg.use_ptw && !sharedSpadXlateCfg.enable && rangeHit
      val ptwHit = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && rangeHit && tlbHit
      val directPaddr = sharedSpadXlateCfg.shared_base + offset
      val translatedPaddr = tlbBase + pageOffset
      val paddr = Mux(passthroughHit, rawAddr, Mux(ptwHit, translatedPaddr, directPaddr))
      val rangeFault = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable && rangeHit && !vpnInBounds
      val waitForXlate = sharedSpadXlateCfg.use_ptw && sharedSpadXlateCfg.enable &&
        rangeHit && vpnInBounds && !tlbHit

      resolved.issueBlocked := rangeFault || waitForXlate
      resolved.needPtwReq := waitForXlate && !pendingHit && !spmPtwPendingValid
      when (rangeFault) {
        assert(false.B, s"GemminiCoupledDMA ${errorLabel} shared-spad vaddr outside programmed PTE range")
      }
      when (directHit || passthroughHit || ptwHit) {
        resolved.paddr := paddr
      }
      resolved.ptwReq.vpn := vpn
      resolved.ptwReq.vaddr := vaddr
      resolved.ptwReq.ptbr := sharedSpadXlateCfg.ptbr
      resolved.ptwReq.pageShift := pageShift
    }

    resolved
  }

  private val beatAddrMask = ~((wideBytes - 1).U(xLenBits.W))
  private val zeroBeatOffset = 0.U(1.W)
  val curSrcBeatOffset = if (beatOffBits > 0) curSrc(beatOffBits - 1, 0) else zeroBeatOffset
  val curDstBeatOffset = if (beatOffBits > 0) curDst(beatOffBits - 1, 0) else zeroBeatOffset
  val curSrcBeatAligned = if (beatOffBits > 0) curSrcBeatOffset === 0.U else true.B
  val curDstBeatAligned = if (beatOffBits > 0) curDstBeatOffset === 0.U else true.B
  val curSrcBeatBase = curSrc & beatAddrMask
  val curDstBeatBase = curDst & beatAddrMask
  val dstBytesLeftInBeat = wideBytes.U(xLenBits.W) - curDstBeatOffset
  val canWide = curSrcBeatAligned && curDstBeatAligned && curRemaining >= wideBytes.U
  val nextXferBytesWide = Mux(canWide, wideBytes.U(xLenBits.W), Mux(curRemaining < dstBytesLeftInBeat, curRemaining, dstBytesLeftInBeat))
  val needSecondRead = !canWide && (curSrcBeatOffset + nextXferBytesWide) > wideBytes.U
  val cacheHitLo = cachedReadValid && (cachedReadBase === curSrcBeatBase)
  val shouldIssueGet0 = state === sIssueGet0 && !cacheHitLo
  val shouldIssueGet1 = state === sIssueGet1
  val issueReadAddr = Mux(state === sIssueGet1, curSrcBeatBase + wideBytes.U, curSrcBeatBase)
  val issueWriteAddr = Mux(state === sIssueFlag, curCompletionAddr, curDstBeatBase)
  val readResolvedAddr = resolveCopyAddr(issueReadAddr, "src")
  val writeResolvedAddr = resolveCopyAddr(issueWriteAddr, "dst")

  when ((shouldIssueGet0 || shouldIssueGet1) && readResolvedAddr.needPtwReq) {
    spmPtwReqValid := true.B
    spmPtwReqBits := readResolvedAddr.ptwReq
  } .elsewhen (state === sIssuePut && writeResolvedAddr.needPtwReq) {
    spmPtwReqValid := true.B
    spmPtwReqBits := writeResolvedAddr.ptwReq
  }

  when (spmPtwReqValid && spmPtwReqReady) {
    spmPtwPendingValid := true.B
    spmPtwPendingVpn := spmPtwReqBits.vpn
  }

  val srcShiftBits = Cat(curSrcBeatOffset, 0.U(3.W))
  val dstShiftBits = Cat(curDstBeatOffset, 0.U(3.W))
  val readWindow = Cat(curReadDataHi, curReadDataLo)
  val copyPutData = ((readWindow >> srcShiftBits)(dataBits - 1, 0) << dstShiftBits)(dataBits - 1, 0)
  val copyPutMask = VecInit((0 until beatBytes).map { i =>
    i.U >= curDstBeatOffset && i.U < (curDstBeatOffset + curXferBytes)
  }).asUInt
  val flagBeatOffset = if (beatOffBits > 0) curCompletionAddr(beatOffBits - 1, 0) else zeroBeatOffset
  val flagByteShift = Cat(flagBeatOffset, 0.U(3.W))
  val flagPutData = (1.U(dataBits.W) << flagByteShift)(dataBits - 1, 0)

  val (_, getBits) = edge.Get(0.U, readResolvedAddr.paddr, wideLgSize)
  val (_, putCopyFullBits) = edge.Put(1.U, writeResolvedAddr.paddr, wideLgSize, copyPutData)
  val (_, putCopyPartialBits) = edge.Put(1.U, writeResolvedAddr.paddr, wideLgSize, copyPutData, copyPutMask)
  val putCopyBits = Wire(chiselTypeOf(putCopyFullBits))
  putCopyBits := Mux(copyPutMask.andR, putCopyFullBits, putCopyPartialBits)
  val (_, putFlagBits) = edge.Put(2.U, curCompletionAddr, 0.U, flagPutData)

  tl.a.valid := false.B
  tl.a.bits := getBits

  when ((shouldIssueGet0 || shouldIssueGet1) &&
      !readResolvedAddr.issueBlocked && !readResolvedAddr.needPtwReq) {
    tl.a.valid := true.B
    tl.a.bits := getBits
  } .elsewhen (state === sIssuePut && !writeResolvedAddr.issueBlocked && !writeResolvedAddr.needPtwReq) {
    tl.a.valid := true.B
    tl.a.bits := putCopyBits
  } .elsewhen (state === sIssueFlag) {
    tl.a.valid := true.B
    tl.a.bits := putFlagBits
  }

  tl.d.ready := state === sWaitGet0 || state === sWaitGet1 || state === sWaitPut || state === sWaitFlag

  when (state === sIdle && copyReqQ.io.deq.valid) {
    copyReqQ.io.deq.ready := true.B
    curSrc := copyReqQ.io.deq.bits.src
    curDst := copyReqQ.io.deq.bits.dst
    curRemaining := copyReqQ.io.deq.bits.len
    curCompletionAddr := copyReqQ.io.deq.bits.completion
    curReadDataLo := 0.U
    curReadDataHi := 0.U
    cachedReadValid := false.B
    state := Mux(copyReqQ.io.deq.bits.len === 0.U, sIssueFlag, sIssueGet0)
  }

  when (state === sIssueGet0) {
    when (cacheHitLo) {
      curXferBytes := nextXferBytesWide(curXferBytes.getWidth - 1, 0)
      curReadDataLo := cachedReadData
      curReadDataHi := 0.U
      state := Mux(needSecondRead, sIssueGet1, sIssuePut)
    } .elsewhen (tl.a.fire) {
      curXferBytes := nextXferBytesWide(curXferBytes.getWidth - 1, 0)
      curReadDataHi := 0.U
      state := sWaitGet0
    }
  }

  when (state === sWaitGet0 && tl.d.fire) {
    curReadDataLo := tl.d.bits.data
    cachedReadValid := true.B
    cachedReadBase := curSrcBeatBase
    cachedReadData := tl.d.bits.data
    state := Mux(needSecondRead, sIssueGet1, sIssuePut)
  }

  when (state === sIssueGet1 && tl.a.fire) {
    state := sWaitGet1
  }

  when (state === sWaitGet1 && tl.d.fire) {
    curReadDataHi := tl.d.bits.data
    cachedReadValid := true.B
    cachedReadBase := curSrcBeatBase + wideBytes.U
    cachedReadData := tl.d.bits.data
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
    state := Mux(nextRemaining === 0.U, sIssueFlag, sIssueGet0)
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
