package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.rocket.HellaCacheArbiter
import freechips.rocketchip.tile.{LazyRoCC, LazyRoCCModuleImp, OpcodeSet, RoCCResponse}
import freechips.rocketchip.tilelink.{TLBuffer, TLIdentityNode, TLXbar}

class GemminiCoupledDMAPairWrapper[T <: Data : Arithmetic, U <: Data, V <: Data](
  baseGemminiConfig: GemminiArrayConfig[T, U, V],
  pairId: Int,
  sharedScratchpadConfig: SharedScratchpadConfig
)(implicit p: Parameters)
    extends LazyRoCC(
      opcodes = OpcodeSet.custom2 | OpcodeSet.custom3,
      nPTWPorts = (if (baseGemminiConfig.use_shared_tlb) 1 else 2)) {

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

  override val tlNode = TLIdentityNode()
  override val atlNode = TLIdentityNode()
  override val stlNode = TLIdentityNode()
  override val sbusSlaveTLNode = gemmini.sbusSlaveTLNode

  tlXbar :=* gemmini.tlNode
  tlXbar :=* dma.tlNode
  tlNode :=* TLBuffer() :=* tlXbar

  atlXbar :=* gemmini.atlNode
  atlXbar :=* dma.atlNode
  atlNode :=* TLBuffer() :=* atlXbar

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

  outer.gemmini.module.io.cmd.valid := io.cmd.valid && isGemminiCmd
  outer.gemmini.module.io.cmd.bits := io.cmd.bits
  outer.dma.module.io.cmd.valid := io.cmd.valid && isDmaCmd
  outer.dma.module.io.cmd.bits := io.cmd.bits

  io.cmd.ready := Mux(isGemminiCmd, outer.gemmini.module.io.cmd.ready,
    Mux(isDmaCmd, outer.dma.module.io.cmd.ready, false.B))

  when (io.cmd.valid) {
    assert(isGemminiCmd || isDmaCmd, "Pair wrapper received an unsupported opcode")
  }

  val respArb = Module(new RRArbiter(new RoCCResponse, 2))
  respArb.io.in(0) <> Queue(outer.gemmini.module.io.resp)
  respArb.io.in(1) <> Queue(outer.dma.module.io.resp)
  io.resp <> respArb.io.out

  io.busy := outer.gemmini.module.io.busy || outer.dma.module.io.busy
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
}
