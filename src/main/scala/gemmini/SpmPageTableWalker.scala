package gemmini

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._

class SpmPtwReq(implicit p: Parameters) extends CoreBundle {
  val vpn = UInt(vaddrBits.W)
  val vaddr = UInt(vaddrBits.W)
  val ptbr = UInt(paddrBits.W)
  val pageShift = UInt(8.W)
}

class SpmPtwResp(implicit p: Parameters) extends CoreBundle {
  val vpn = UInt(vaddrBits.W)
  val vaddr = UInt(vaddrBits.W)
  val pte = UInt(64.W)
  val accessFault = Bool()
  val pageShift = UInt(8.W)
}

class SpmPageTableWalker(name: String)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    name = name,
    sourceId = IdRange(0, 1),
    requestFifo = true
  )), minLatency = 1)))

  lazy val module = new Impl

  class Impl extends LazyModuleImp(this) with HasCoreParameters {
    val io = IO(new Bundle {
      val req = Flipped(Decoupled(new SpmPtwReq))
      val resp = Valid(new SpmPtwResp)
      val busy = Output(Bool())
    })

    val (tl, edge) = node.out(0)

    tl.b.ready := true.B
    tl.c.valid := false.B
    tl.c.bits := DontCare
    tl.e.valid := false.B
    tl.e.bits := DontCare

    val beatBytes = tl.params.dataBits / 8
    require(beatBytes >= 8, s"SPM PTW requires TileLink beat >= 8B, got $beatBytes")
    val beatOffBits = log2Ceil(beatBytes)

    val sIdle :: sIssueGet :: sWaitGet :: Nil = Enum(3)
    val state = RegInit(sIdle)

    val reqReg = Reg(new SpmPtwReq)
    val reqAddrReg = Reg(UInt(paddrBits.W))

    val (_, getBits) = edge.Get(
      fromSource = 0.U,
      toAddress = reqAddrReg,
      lgSize = log2Ceil(8).U
    )

    tl.a.valid := state === sIssueGet
    tl.a.bits := getBits
    tl.d.ready := state === sWaitGet

    io.req.ready := state === sIdle
    io.busy := state =/= sIdle

    io.resp.valid := false.B
    io.resp.bits := 0.U.asTypeOf(new SpmPtwResp)

    when (io.req.fire) {
      reqReg := io.req.bits
      reqAddrReg := io.req.bits.ptbr + (io.req.bits.vpn << 3)
      state := sIssueGet
    }

    when (state === sIssueGet && tl.a.fire) {
      state := sWaitGet
    }

    when (state === sWaitGet && tl.d.fire) {
      val dataShift = if (beatOffBits > 0) {
        Cat(reqAddrReg(beatOffBits - 1, 0), 0.U(3.W))
      } else {
        0.U(3.W)
      }
      val pteWord = (tl.d.bits.data >> dataShift)(63, 0)

      io.resp.valid := true.B
      io.resp.bits.vpn := reqReg.vpn
      io.resp.bits.vaddr := reqReg.vaddr
      io.resp.bits.pte := pteWord
      io.resp.bits.accessFault := tl.d.bits.denied || tl.d.bits.corrupt
      io.resp.bits.pageShift := reqReg.pageShift
      state := sIdle
    }
  }
}
