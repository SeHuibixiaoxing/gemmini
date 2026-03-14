package gemmini

import chisel3._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.CoreBundle

class SharedSpadXlateConfig(implicit p: Parameters) extends CoreBundle {
  val use_ptw = Bool()
  val enable = Bool()
  val page_shift = UInt(8.W)
  val pte_count = UInt(16.W)
  val ptbr = UInt(paddrBits.W)
  val range_base = UInt(vaddrBits.W)
  val range_size = UInt(vaddrBits.W)
  val shared_base = UInt(paddrBits.W)
  val cache_epoch = UInt(8.W)
}
