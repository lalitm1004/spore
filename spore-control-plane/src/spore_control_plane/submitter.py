"""Order dispatch: hand an order to any reachable bot and let the fleet route it.

WHAT
    `OrderSubmitter.dispatch(order)` sends the `Order` to the bots in
    `BOT_ADDRESSES`, retrying `DISPATCH_ATTEMPTS` times, until one accepts.
    It knows nothing about regions or leaders — the fleet does the routing.

WHERE
    Called by the web layer for each POST /orders. Holds the persistent channel
    pool via `client.pool`.

WHY
    The control plane can never know which region a bot is in, or who leads
    what: bots migrate and leaders rotate. The only correct place to resolve
    pickup_node -> region -> leader is inside the fleet's own dispatcher, at
    dispatch time. So we just hand the order to any bot and let it forward.

HOW
    * `dispatch` tries every bot address; on failure it backs off and tries
      again, up to `DISPATCH_ATTEMPTS`. Raises `DispatchError` when exhausted.
    * Retries use the *same* `order_id`, which is the idempotency key: the fleet
      dedupes by it, so a retry after a timeout cannot double-place the order.
"""
from __future__ import annotations

import logging
import time

import grpc

from spore_control_plane import config
from spore_control_plane import client
from spore_control_plane.proto import controlplane_pb2, controlplane_pb2_grpc

log = logging.getLogger(__name__)


class DispatchError(Exception):
    """No bot accepted the order within `DISPATCH_ATTEMPTS` passes."""

    def __init__(self, order_id: str, attempts: int) -> None:
        super().__init__(f"order {order_id}: not dispatched after {attempts} attempt(s)")
        self.order_id = order_id
        self.attempts = attempts


class OrderSubmitter:
    def __init__(self, addresses: list[str] | None = None) -> None:
        self._addresses = list(addresses if addresses is not None else config.BOT_ADDRESSES)

    def dispatch(self, order: controlplane_pb2.Order) -> controlplane_pb2.DispatchAck:
        """Try every known bot, backing off and retrying, until one accepts."""
        for attempt in range(1, config.DISPATCH_ATTEMPTS + 1):
            for address in self._addresses:
                ack = self._call(address, order)
                if ack is not None and ack.accepted:
                    log.info("order %s accepted via %s (owner_region=%d, assignee=%s)",
                             order.order_id, address, ack.owner_region,
                             ack.assignee if ack.HasField("assignee") else "-")
                    return ack
            if attempt < config.DISPATCH_ATTEMPTS:
                time.sleep(config.DISPATCH_BACKOFF)

        log.error("order %s: giving up after %d attempt(s) across %d bot(s)",
                  order.order_id, config.DISPATCH_ATTEMPTS, len(self._addresses))
        raise DispatchError(order.order_id, config.DISPATCH_ATTEMPTS)

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
