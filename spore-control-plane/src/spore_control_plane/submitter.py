"""Order dispatch: turn an order into a gRPC call to the right leader.

WHAT
    `OrderSubmitter.dispatch` sends an `Order` to the fleet and returns the
    `DispatchAck`. It prefers the pickup-region's leader (when known) and falls
    back to any bot, because the fleet forwards an order to the right leader
    anyway.

WHERE
    Called by the web layer for each POST /orders. Holds the persistent channel
    pool via `client.pool`.

WHY
    Direct-to-leader is faster and matches the product requirement ("send the
    job to the leader of the region where it starts"); the fallback keeps
    dispatch working when discovery is stale or the leader is unreachable.

HOW
    * `dispatch(order, region_id)` builds a candidate list — the region's
      leader first, then every known bot (deduped) — and tries each in turn.
    * Retries use the *same* `order_id`, which is the idempotency key: the
      fleet dedupes by it, so a retry after a timeout cannot double-place the
      order.
"""
from __future__ import annotations

import logging

import grpc

from spore_control_plane import config
from spore_control_plane import client
from spore_control_plane.proto import controlplane_pb2, controlplane_pb2_grpc

log = logging.getLogger(__name__)


class DispatchError(Exception):
    """Every candidate bot failed to accept the order."""

    def __init__(self, order_id: str, failures: int) -> None:
        super().__init__(f"order {order_id}: no bot accepted the dispatch ({failures} attempts)")
        self.order_id = order_id
        self.failures = failures


class OrderSubmitter:
    def __init__(self, addresses: list[str] | None = None) -> None:
        self._addresses = list(addresses if addresses is not None else config.BOT_ADDRESSES)

    def dispatch(self, order: controlplane_pb2.Order, region_id: int | None,
                 leader_address: str | None = None) -> controlplane_pb2.DispatchAck:
        """Try the pickup-region leader first, then every known bot. Raises
        `DispatchError` if nobody accepts."""
        candidates: list[str] = []
        if leader_address:
            candidates.append(leader_address)
        candidates.extend(a for a in self._addresses if a != leader_address)

        failures = 0
        last_note = "no bots configured"
        for address in candidates:
            ack = self._call(address, order)
            if ack is not None:
                if ack.accepted:
                    log.info("order %s dispatched via %s (owner_region=%d, assignee=%s)",
                             order.order_id, address, ack.owner_region,
                             ack.assignee if ack.HasField("assignee") else "-")
                    return ack
                last_note = ack.note
            failures += 1

        log.error("order %s rejected by %d bot(s): %s", order.order_id, failures, last_note)
        raise DispatchError(order.order_id, failures)

    def _call(self, address: str, order: controlplane_pb2.Order) -> controlplane_pb2.DispatchAck | None:
        try:
            stub = client.pool.stub(address, controlplane_pb2_grpc.ControlPlaneServiceStub)
            return stub.DispatchOrder(
                order, timeout=config.GRPC_TIMEOUT, metadata=client.metadata(),
                wait_for_ready=True,
            )
        except grpc.RpcError as e:
            log.debug("DispatchOrder to %s failed: %s", address, e.code())
            return None
