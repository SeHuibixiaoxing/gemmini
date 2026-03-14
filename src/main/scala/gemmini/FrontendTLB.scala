package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile.{CoreBundle, CoreModule}
import freechips.rocketchip.tilelink.TLEdgeOut

import Util._

import midas.targetutils.PerfCounter

class DecoupledTLBReq(val lgMaxSize: Int)(implicit p: Parameters) extends CoreBundle {
  val tlb_req = new TLBReq(lgMaxSize)
  val status = new MStatus
}

class TLBExceptionIO extends Bundle {
  val interrupt = Output(Bool())
  val flush_retry = Input(Bool())
  val flush_skip = Input(Bool())

  def flush(dummy: Int = 0): Bool = flush_retry || flush_skip
}

// TODO can we make TLB hits only take one cycle?
class DecoupledTLB(entries: Int, maxSize: Int, use_firesim_simulation_counters: Boolean)(implicit edge: TLEdgeOut, p: Parameters)
  extends CoreModule {

  val lgMaxSize = log2Ceil(maxSize)
  val io = IO(new Bundle {
    val req = Flipped(Valid(new DecoupledTLBReq(lgMaxSize)))
    val resp = new TLBResp
    val ptw = new TLBPTWIO

    val exp = new TLBExceptionIO

    val counter = new CounterEventIO()
  })

  val interrupt = RegInit(false.B)
  io.exp.interrupt := interrupt

  val tlb = Module(new TLB(false, lgMaxSize, TLBConfig(nSets=1, nWays=entries)))
  tlb.io.req.valid := io.req.valid
  tlb.io.req.bits := io.req.bits.tlb_req
  io.resp := tlb.io.resp
  tlb.io.kill := false.B

  tlb.io.sfence.valid := io.exp.flush()
  tlb.io.sfence.bits.rs1 := false.B
  tlb.io.sfence.bits.rs2 := false.B
  tlb.io.sfence.bits.addr := DontCare
  tlb.io.sfence.bits.asid := DontCare
  tlb.io.sfence.bits.hv := false.B
  tlb.io.sfence.bits.hg := false.B

  io.ptw <> tlb.io.ptw
  tlb.io.ptw.status := io.req.bits.status
  val exception = io.req.valid && Mux(io.req.bits.tlb_req.cmd === M_XRD, tlb.io.resp.pf.ld || tlb.io.resp.ae.ld, tlb.io.resp.pf.st || tlb.io.resp.ae.st)
  when (exception) { interrupt := true.B }
  when (interrupt && tlb.io.sfence.fire) {
    interrupt := false.B
  }

  assert(!io.exp.flush_retry || !io.exp.flush_skip, "TLB: flushing with both retry and skip at same time")

  CounterEventIO.init(io.counter)
  io.counter.connectEventSignal(CounterEvent.DMA_TLB_HIT_REQ, io.req.fire && !tlb.io.resp.miss)
  io.counter.connectEventSignal(CounterEvent.DMA_TLB_TOTAL_REQ, io.req.fire)
  io.counter.connectEventSignal(CounterEvent.DMA_TLB_MISS_CYCLE, tlb.io.resp.miss)

  if (use_firesim_simulation_counters) {
    PerfCounter(io.req.fire && !tlb.io.resp.miss, "tlb_hits", "total number of tlb hits")
    PerfCounter(io.req.fire, "tlb_reqs", "total number of tlb reqs")
    PerfCounter(tlb.io.resp.miss, "tlb_miss_cycles", "total number of cycles where the tlb is resolving a miss")
  }
}

class FrontendTLBIO(implicit p: Parameters) extends CoreBundle {
  val lgMaxSize = log2Ceil(coreDataBytes)
  val req = Valid(new DecoupledTLBReq(lgMaxSize))
  val resp = Flipped(new TLBResp)
}

class FrontendTLB(nClients: Int, entries: Int, maxSize: Int, use_tlb_register_filter: Boolean, use_firesim_simulation_counters: Boolean, use_shared_tlb: Boolean)
                 (implicit edge: TLEdgeOut, p: Parameters) extends CoreModule {

  val num_tlbs = if (use_shared_tlb) 1 else nClients
  val lgMaxSize = log2Ceil(coreDataBytes)
  val spmRefillIdxWidth = log2Ceil(entries max 2)

  val io = IO(new Bundle {
    val clients = Flipped(Vec(nClients, new FrontendTLBIO))
    val ptw = Vec(num_tlbs, new TLBPTWIO)
    val exp = Vec(num_tlbs, new TLBExceptionIO)
    val counter = new CounterEventIO()

    // Shared-spad translation control.
    val spm_use_ptw = Input(Bool())
    val spm_xlate_enable = Input(Bool())
    val spm_xlate_page_shift = Input(UInt(8.W))
    val spm_xlate_pte_count = Input(UInt(16.W))
    val spm_xlate_ptbr = Input(UInt(paddrBits.W))
    val spm_xlate_range_base = Input(UInt(vaddrBits.W))
    val spm_xlate_range_size = Input(UInt(vaddrBits.W))
    val spm_xlate_shared_base = Input(UInt(paddrBits.W))
    val spm_cache_clear = Input(Bool())
    val spm_fault_clear = Input(Bool())
    val spm_fault_valid = Output(Bool())
    val spm_fault_vaddr = Output(UInt(vaddrBits.W))
    val spm_fault_cause = Output(UInt(8.W))
    val spm_ptw_req = Decoupled(new SpmPtwReq)
    val spm_ptw_resp = Flipped(Valid(new SpmPtwResp))
  })

  val spm_fault_valid = RegInit(false.B)
  val spm_fault_vaddr = RegInit(0.U(vaddrBits.W))
  val spm_fault_cause = RegInit(0.U(8.W))
  io.spm_fault_valid := spm_fault_valid
  io.spm_fault_vaddr := spm_fault_vaddr
  io.spm_fault_cause := spm_fault_cause
  when (io.spm_fault_clear) {
    spm_fault_valid := false.B
    spm_fault_vaddr := 0.U
    spm_fault_cause := 0.U
  }

  val tlbs = Seq.fill(num_tlbs)(Module(new DecoupledTLB(entries, maxSize, use_firesim_simulation_counters)))

  io.ptw <> VecInit(tlbs.map(_.io.ptw))
  io.exp <> VecInit(tlbs.map(_.io.exp))

  val tlbArbOpt = if (use_shared_tlb) Some(Module(new RRArbiter(new DecoupledTLBReq(lgMaxSize), nClients))) else None

  if (use_shared_tlb) {
    val tlbArb = tlbArbOpt.get
    val tlb = tlbs.head
    tlb.io.req.valid := tlbArb.io.out.valid
    tlb.io.req.bits := tlbArb.io.out.bits
    tlbArb.io.out.ready := true.B
  }

  val spmTlbValid = RegInit(VecInit(Seq.fill(entries)(false.B)))
  val spmTlbVpn = Reg(Vec(entries, UInt(vaddrBits.W)))
  val spmTlbPaddrBase = Reg(Vec(entries, UInt(paddrBits.W)))
  val spmRefillPtr = RegInit(0.U(spmRefillIdxWidth.W))

  val spmPtwPendingValid = RegInit(false.B)
  val spmPtwPendingVpn = RegInit(0.U(vaddrBits.W))
  val spmPtwPendingVaddr = RegInit(0.U(vaddrBits.W))

  val tlbFlush = tlbs.map(_.io.exp.flush()).reduce(_ || _)
  val spmCacheInvalidate = io.spm_cache_clear || tlbFlush
  when (spmCacheInvalidate) {
    spmTlbValid.foreach(_ := false.B)
    spmPtwPendingValid := false.B
  }

  when (io.spm_ptw_resp.valid && spmPtwPendingValid) {
    spmPtwPendingValid := false.B

    when (io.spm_ptw_resp.bits.accessFault) {
      when (!spm_fault_valid) {
        spm_fault_valid := true.B
        spm_fault_vaddr := io.spm_ptw_resp.bits.vaddr
        spm_fault_cause := 3.U // shared-spad PTE access fault
      }
    }.elsewhen (!io.spm_ptw_resp.bits.pte(0)) {
      when (!spm_fault_valid) {
        spm_fault_valid := true.B
        spm_fault_vaddr := io.spm_ptw_resp.bits.vaddr
        spm_fault_cause := 2.U // invalid shared-spad PTE
      }
    }.otherwise {
      val refillIdx = spmRefillPtr
      val paddrBase = ((io.spm_ptw_resp.bits.pte(63, 1).asUInt << io.spm_ptw_resp.bits.pageShift)(paddrBits - 1, 0))
      spmTlbValid(refillIdx) := true.B
      spmTlbVpn(refillIdx) := io.spm_ptw_resp.bits.vpn
      spmTlbPaddrBase(refillIdx) := paddrBase
      if (entries > 1) {
        spmRefillPtr := Mux(spmRefillPtr === (entries - 1).U, 0.U, spmRefillPtr + 1.U)
      }
    }
  }

  val spmPtwReqValid = WireInit(VecInit(Seq.fill(nClients)(false.B)))
  val spmPtwReqBits = Wire(Vec(nClients, new SpmPtwReq))
  spmPtwReqBits.foreach(_ := 0.U.asTypeOf(new SpmPtwReq))

  io.clients.zipWithIndex.foreach { case (client, i) =>
    val lastDramTranslatedValid = RegInit(false.B)
    val lastDramTranslatedVpn = RegInit(0.U(vaddrBits.W))
    val lastDramTranslatedPaddr = RegInit(0.U(paddrBits.W))

    val reqVaddr = client.req.bits.tlb_req.vaddr
    val spmRangeEnd = io.spm_xlate_range_base + io.spm_xlate_range_size
    val spmRangeHit = (io.spm_xlate_range_size =/= 0.U) &&
      reqVaddr >= io.spm_xlate_range_base && reqVaddr < spmRangeEnd
    // Shared-spad translation uses its own software-programmed page size; do not clamp it to the CPU\x27s 4 KiB page size.
    val spmPageShift = io.spm_xlate_page_shift
    val spmOffset = reqVaddr - io.spm_xlate_range_base
    val spmVpn = spmOffset >> spmPageShift
    val spmVpnInBounds = spmVpn < io.spm_xlate_pte_count
    val spmPageOffset = spmOffset - (spmVpn << spmPageShift)

    val spmTlbHits = VecInit(spmTlbValid.zip(spmTlbVpn).map { case (valid, vpn) => valid && vpn === spmVpn })
    val spmTlbHit = spmTlbHits.asUInt.orR
    val spmTlbBase = Mux(spmTlbHit, Mux1H(spmTlbHits, spmTlbPaddrBase), 0.U(paddrBits.W))
    val spmPendingHit = spmPtwPendingValid && spmPtwPendingVpn === spmVpn

    val useSpmPtw = io.spm_use_ptw && spmRangeHit
    val spmDirectHit = !io.spm_use_ptw && io.spm_xlate_enable && spmRangeHit && spmVpnInBounds
    val spmPassthroughHit = io.spm_use_ptw && !io.spm_xlate_enable && spmRangeHit
    val spmPtwHit = io.spm_use_ptw && io.spm_xlate_enable && spmRangeHit && spmTlbHit
    val spmHit = spmDirectHit || spmPassthroughHit || spmPtwHit

    val spmDirectPaddr = io.spm_xlate_shared_base + spmOffset
    val spmTranslatedPaddr = spmTlbBase + spmPageOffset
    val spmPaddr = Mux(spmPassthroughHit, reqVaddr, Mux(spmPtwHit, spmTranslatedPaddr, spmDirectPaddr))

    val dramL0Hit = lastDramTranslatedValid &&
      ((reqVaddr >> pgIdxBits).asUInt === (lastDramTranslatedVpn >> pgIdxBits).asUInt)
    val cachedDramPaddr = Cat(lastDramTranslatedPaddr >> pgIdxBits, reqVaddr(pgIdxBits - 1, 0))
    val l0TlbHit = dramL0Hit || spmHit
    val l0TlbPaddr = Mux(spmHit, spmPaddr, cachedDramPaddr)

    val tlb = if (use_shared_tlb) tlbs.head else tlbs(i)
    val tlbReq = if (use_shared_tlb) tlbArbOpt.get.io.in(i).bits else tlb.io.req.bits
    val tlbReqValid = if (use_shared_tlb) tlbArbOpt.get.io.in(i).valid else tlb.io.req.valid
    val tlbReqFire = if (use_shared_tlb) tlbArbOpt.get.io.in(i).fire else tlb.io.req.fire

    tlbReqValid := RegNext(client.req.valid && !dramL0Hit && !spmRangeHit)
    tlbReq := RegNext(client.req.bits)

    when (tlbReqFire && !tlb.io.resp.miss) {
      lastDramTranslatedValid := true.B
      lastDramTranslatedVpn := tlbReq.tlb_req.vaddr
      lastDramTranslatedPaddr := tlb.io.resp.paddr
    }

    when (spmCacheInvalidate) {
      lastDramTranslatedValid := false.B
    }

    when (client.req.valid && !io.spm_use_ptw && io.spm_xlate_enable && spmRangeHit && !spmVpnInBounds && !spm_fault_valid) {
      spm_fault_valid := true.B
      spm_fault_vaddr := reqVaddr
      spm_fault_cause := 2.U // legacy direct translation out-of-range
    }

    when (client.req.valid && io.spm_use_ptw && io.spm_xlate_enable && spmRangeHit && !spmVpnInBounds && !spm_fault_valid) {
      spm_fault_valid := true.B
      spm_fault_vaddr := reqVaddr
      spm_fault_cause := 1.U // shared-spad vaddr outside programmed PTE range
    }

    spmPtwReqValid(i) := client.req.valid && useSpmPtw && io.spm_xlate_enable && spmVpnInBounds && !spmTlbHit && !spmPendingHit && !spmPtwPendingValid
    spmPtwReqBits(i).vpn := spmVpn
    spmPtwReqBits(i).vaddr := reqVaddr
    spmPtwReqBits(i).ptbr := io.spm_xlate_ptbr
    spmPtwReqBits(i).pageShift := spmPageShift

    when (tlbReqFire) {
      client.resp := tlb.io.resp
    }.otherwise {
      client.resp := DontCare
      client.resp.paddr := RegNext(l0TlbPaddr)
      client.resp.miss := !RegNext(l0TlbHit)
    }

    if (!use_tlb_register_filter) {
      lastDramTranslatedValid := false.B
    }
  }

  val spmPtwArb = Module(new RRArbiter(new SpmPtwReq, nClients))
  spmPtwArb.io.in.zipWithIndex.foreach { case (in, i) =>
    in.valid := spmPtwReqValid(i)
    in.bits := spmPtwReqBits(i)
  }
  io.spm_ptw_req.valid := spmPtwArb.io.out.valid
  io.spm_ptw_req.bits := spmPtwArb.io.out.bits
  spmPtwArb.io.out.ready := io.spm_ptw_req.ready

  when (io.spm_ptw_req.fire) {
    spmPtwPendingValid := true.B
    spmPtwPendingVpn := io.spm_ptw_req.bits.vpn
    spmPtwPendingVaddr := io.spm_ptw_req.bits.vaddr
  }

  io.counter := DontCare
  tlbs.foreach(_.io.counter.external_reset := false.B)
  io.counter.collect(tlbs.head.io.counter)
}
