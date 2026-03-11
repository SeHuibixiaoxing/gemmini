package gemmini

import scala.collection.mutable

import freechips.rocketchip.tilelink.TLOutwardNode

/**
 * Registry used during diplomacy graph construction to connect a coupled DMA
 * client to the corresponding Gemmini shared scratchpad local ingress.
 *
 * Gemmini managers register their local shared-spad node by gemmini_id.
 * Coupled DMA managers request a connection by the same gemmini_id.
 */
object CoupledSharedSpadRegistry {
  private val localConnectors = mutable.HashMap[Int, TLOutwardNode => Unit]()
  private val pendingClients = mutable.HashMap[Int, List[TLOutwardNode]]()

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

  def hasLocalNode(gemminiId: Int): Boolean = synchronized { localConnectors.contains(gemminiId) }
}
