package gemmini

import scala.collection.mutable

import freechips.rocketchip.diplomacy.BundleBridgeSink
import freechips.rocketchip.tilelink.TLOutwardNode

/**
 * Registry used during diplomacy graph construction to connect a coupled DMA
 * client to the corresponding Gemmini shared scratchpad local ingress and,
 * when enabled, to the per-gemmini shared-spad translation context.
 */
object CoupledSharedSpadRegistry {
  private val localConnectors = mutable.HashMap[Int, TLOutwardNode => Unit]()
  private val pendingClients = mutable.HashMap[Int, List[TLOutwardNode]]()

  private val xlateConnectors = mutable.HashMap[Int, BundleBridgeSink[SharedSpadXlateConfig] => Unit]()
  private val pendingXlateClients = mutable.HashMap[Int, List[BundleBridgeSink[SharedSpadXlateConfig]]]()

  def registerLocalNode(gemminiId: Int, connector: TLOutwardNode => Unit): Seq[TLOutwardNode] = synchronized {
    localConnectors(gemminiId) = connector
    pendingClients.remove(gemminiId).getOrElse(Nil).reverse
  }

  def connectClient(gemminiId: Int, client: TLOutwardNode): Boolean = synchronized {
    localConnectors.get(gemminiId) match {
      case Some(connector) =>
        connector(client)
        true
      case None =>
        val current = pendingClients.getOrElse(gemminiId, Nil)
        pendingClients(gemminiId) = client :: current
        false
    }
  }

  def registerXlateNode(gemminiId: Int,
                        connector: BundleBridgeSink[SharedSpadXlateConfig] => Unit)
      : Seq[BundleBridgeSink[SharedSpadXlateConfig]] = synchronized {
    xlateConnectors(gemminiId) = connector
    pendingXlateClients.remove(gemminiId).getOrElse(Nil).reverse
  }

  def connectXlateClient(gemminiId: Int,
                         client: BundleBridgeSink[SharedSpadXlateConfig]): Boolean = synchronized {
    xlateConnectors.get(gemminiId) match {
      case Some(connector) =>
        connector(client)
        true
      case None =>
        val current = pendingXlateClients.getOrElse(gemminiId, Nil)
        pendingXlateClients(gemminiId) = client :: current
        false
    }
  }

  def hasLocalNode(gemminiId: Int): Boolean = synchronized { localConnectors.contains(gemminiId) }
  def hasXlateNode(gemminiId: Int): Boolean = synchronized { xlateConnectors.contains(gemminiId) }
}
