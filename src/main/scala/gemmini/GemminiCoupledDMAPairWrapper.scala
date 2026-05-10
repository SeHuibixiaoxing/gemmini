package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.rocket.HellaCacheArbiter
import freechips.rocketchip.tile.{LazyRoCC, LazyRoCCModuleImp, OpcodeSet, RoCCResponse}
import freechips.rocketchip.tilelink.{TLBuffer, TLIdentityNode, TLSourceShrinker, TLXbar}
import midas.targetutils.{PerfCounter, SynthesizePrintf}

class GemminiCoupledDMAPairWrapper[T <: Data : Arithmetic, U <: Data, V <: Data](
  baseGemminiConfig: GemminiArrayConfig[T, U, V],
  pairId: Int,
  sharedScratchpadConfig: SharedScratchpadConfig,
  tlMaxInFlight: Option[Int] = None,
  atlMaxInFlight: Option[Int] = None
)(implicit p: Parameters)
    extends LazyRoCC(
      opcodes = OpcodeSet.custom2 | OpcodeSet.custom3,
      nPTWPorts = (if (baseGemminiConfig.use_shared_tlb) 1 else 2)) {
  val debugPairId = pairId

  tlMaxInFlight.foreach(v => require(v > 0, s"tlMaxInFlight must be > 0 when set, got $v"))
  atlMaxInFlight.foreach(v => require(v > 0, s"atlMaxInFlight must be > 0 when set, got $v"))

  private val gemminiConfig = baseGemminiConfig.copy(
    opcodes = OpcodeSet.custom3,
    gemmini_id = pairId,
    shared_scratchpad_config = sharedScratchpadConfig
  )

  private val dmaParams = CoupledDMAParams(
    gemmini_id = pairId,
    shared_scratchpad_config = sharedScratchpadConfig
  )

  val gemmini = LazyModule(new Gemmini(gemminiConfig))
  val dma = LazyModule(new GemminiCoupledDMA(OpcodeSet.custom2, dmaParams))

  private val tlXbar = TLXbar()
  private val atlXbar = TLXbar()
  private val stlXbar = TLXbar()
  private val tlShrinkNode = tlMaxInFlight.map(TLSourceShrinker(_)).getOrElse(TLIdentityNode())
  private val atlShrinkNode = atlMaxInFlight.map(TLSourceShrinker(_)).getOrElse(TLIdentityNode())

  override val tlNode = TLIdentityNode()
  override val atlNode = TLIdentityNode()
  override val stlNode = TLIdentityNode()
  override val sbusSlaveTLNode = gemmini.sbusSlaveTLNode

  tlXbar :=* gemmini.tlNode
  tlXbar :=* dma.tlNode
  tlNode :=* TLBuffer() :=* tlShrinkNode :=* tlXbar

  atlXbar :=* gemmini.atlNode
  atlXbar :=* dma.atlNode
  atlNode :=* TLBuffer() :=* atlShrinkNode :=* atlXbar

  gemmini.stlNode :*= stlXbar
  dma.stlNode :*= stlXbar
  stlXbar :*= TLBuffer() :*= stlNode

  override lazy val module = new GemminiCoupledDMAPairWrapperModule(this)
}

class GemminiCoupledDMAPairWrapperModule[T <: Data : Arithmetic, U <: Data, V <: Data](
  outer: GemminiCoupledDMAPairWrapper[T, U, V]
)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) {

  private val gemminiOpcode = OpcodeSet.custom3.opcodes.head
  private val dmaOpcode = OpcodeSet.custom2.opcodes.head
  private val gemminiPtwPorts = outer.gemmini.nPTWPorts
  private val dmaPtwPorts = outer.dma.nPTWPorts

  private val isGemminiCmd = io.cmd.bits.inst.opcode === gemminiOpcode
  private val isDmaCmd = io.cmd.bits.inst.opcode === dmaOpcode

  private def ageWhile(active: Bool, width: Int = 32): UInt = {
    val age = RegInit(0.U(width.W))
    when (active) {
      when (!age.andR) { age := age + 1.U }
    } .otherwise {
      age := 0.U
    }
    age
  }

  private def debugStuckPrint(age: UInt): Bool =
    age === 1024.U || (age > 1024.U && age(11, 0) === 0.U)

  outer.gemmini.module.io.cmd.valid := io.cmd.valid && isGemminiCmd
  outer.gemmini.module.io.cmd.bits := io.cmd.bits
  outer.dma.module.io.cmd.valid := io.cmd.valid && isDmaCmd
  outer.dma.module.io.cmd.bits := io.cmd.bits

  io.cmd.ready := Mux(isGemminiCmd, outer.gemmini.module.io.cmd.ready,
    Mux(isDmaCmd, outer.dma.module.io.cmd.ready, false.B))

  val cmdBlockedAge = ageWhile(io.cmd.valid && !io.cmd.ready)
  when (debugStuckPrint(cmdBlockedAge)) {
    SynthesizePrintf(printf(
      "[pair-wrapper-stuck] pair=%d age=%d valid=%d ready=%d is_gemmini=%d is_dma=%d gemmini_ready=%d dma_ready=%d gemmini_busy=%d dma_busy=%d opc=%x funct=%x\n",
      outer.debugPairId.U, cmdBlockedAge, io.cmd.valid.asUInt, io.cmd.ready.asUInt,
      isGemminiCmd.asUInt, isDmaCmd.asUInt, outer.gemmini.module.io.cmd.ready.asUInt,
      outer.dma.module.io.cmd.ready.asUInt, outer.gemmini.module.io.busy.asUInt,
      outer.dma.module.io.busy.asUInt, io.cmd.bits.inst.opcode, io.cmd.bits.inst.funct))
  }

  when (outer.gemmini.module.io.cmd.fire) {
    SynthesizePrintf(printf(
      "[pair-wrapper-gemmini-cmd] pair=%d funct=%x rd=%d rs1=%x rs2=%x busy=%d\n",
      outer.debugPairId.U, io.cmd.bits.inst.funct, io.cmd.bits.inst.rd,
      io.cmd.bits.rs1, io.cmd.bits.rs2, outer.gemmini.module.io.busy.asUInt))
  }

  when (outer.dma.module.io.cmd.fire) {
    SynthesizePrintf(printf(
      "[pair-wrapper-dma-cmd] pair=%d funct=%x rd=%d rs1=%x rs2=%x busy=%d\n",
      outer.debugPairId.U, io.cmd.bits.inst.funct, io.cmd.bits.inst.rd,
      io.cmd.bits.rs1, io.cmd.bits.rs2, outer.dma.module.io.busy.asUInt))
  }

  when (io.cmd.valid) {
    assert(isGemminiCmd || isDmaCmd, "Pair wrapper received an unsupported opcode")
  }

  val respArb = Module(new RRArbiter(new RoCCResponse, 2))
  respArb.io.in(0) <> Queue(outer.gemmini.module.io.resp)
  respArb.io.in(1) <> Queue(outer.dma.module.io.resp)
  io.resp <> respArb.io.out
  val pairBusy = outer.gemmini.module.io.busy || outer.dma.module.io.busy

  when (io.resp.fire) {
    SynthesizePrintf(printf(
      "[pair-wrapper-resp] pair=%d rd=%d data=%x gemmini_valid=%d dma_valid=%d busy=%d\n",
      outer.debugPairId.U, io.resp.bits.rd, io.resp.bits.data,
      respArb.io.in(0).valid.asUInt, respArb.io.in(1).valid.asUInt,
      pairBusy.asUInt))
  }

  io.busy := pairBusy
  io.interrupt := outer.gemmini.module.io.interrupt || outer.dma.module.io.interrupt

  val dcacheArb = Module(new HellaCacheArbiter(2))
  dcacheArb.io.requestor(0) <> outer.gemmini.module.io.mem
  dcacheArb.io.requestor(1) <> outer.dma.module.io.mem
  io.mem <> dcacheArb.io.mem

  for (i <- 0 until gemminiPtwPorts) {
    outer.gemmini.module.io.ptw(i) <> io.ptw(i)
  }
  for (i <- 0 until dmaPtwPorts) {
    outer.dma.module.io.ptw(i) <> io.ptw(gemminiPtwPorts + i)
  }

  outer.gemmini.module.io.exception := io.exception
  outer.dma.module.io.exception := io.exception

  io.fpu_req.valid := false.B
  io.fpu_req.bits := DontCare
  io.fpu_resp.ready := false.B

  outer.gemmini.module.io.fpu_req.ready := false.B
  assert(!outer.gemmini.module.io.fpu_req.valid)
  outer.gemmini.module.io.fpu_resp.valid := false.B
  outer.gemmini.module.io.fpu_resp.bits := DontCare

  outer.dma.module.io.fpu_req.ready := false.B
  assert(!outer.dma.module.io.fpu_req.valid)
  outer.dma.module.io.fpu_resp.valid := false.B
  outer.dma.module.io.fpu_resp.bits := DontCare

  val debugState = Cat(
    io.cmd.valid,
    io.cmd.ready,
    isGemminiCmd,
    isDmaCmd,
    outer.gemmini.module.io.cmd.ready,
    outer.dma.module.io.cmd.ready,
    outer.gemmini.module.io.busy,
    outer.dma.module.io.busy,
    io.resp.valid,
    io.resp.ready,
    respArb.io.in(0).valid,
    respArb.io.in(1).valid)
  PerfCounter.identity(debugState, s"gemmini_dma_pair_${outer.debugPairId}_state",
    "Gemmini/CoupledDMA pair wrapper command routing state")
  PerfCounter(outer.gemmini.module.io.cmd.fire.asUInt, s"gemmini_dma_pair_${outer.debugPairId}_gemmini_cmd",
    "Pair wrapper forwarded a Gemmini command")
  PerfCounter(outer.dma.module.io.cmd.fire.asUInt, s"gemmini_dma_pair_${outer.debugPairId}_dma_cmd",
    "Pair wrapper forwarded a CoupledDMA command")
}
