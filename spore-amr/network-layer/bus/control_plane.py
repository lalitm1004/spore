"""`ControlPlaneService` — where cargo orders enter the fleet.

WHAT
    `DispatchOrder(Order) -> DispatchAck`, served by **every** bot. One order
    in, and the fleet works out whose it is.

WHERE
    Registered on every bot's gRPC server alongside `JobService`. The caller is
    `spore-control-plane`, which holds a list of bot addresses and tries them in
    turn until one accepts.

WHY there are two doors into the same room
    `JobService.SubmitJob` already does this, and takes a `fleet.Job`. That is
    the fleet's own shape: it carries `owner_region`, `assignee`, `status`,
    `last_node` — fields an order system has no business knowing about and no
    way to fill in. Making the control plane speak it would couple an external
    system to our internal wire, and every future field we added to `Job` would
    be a field it had to be taught to ignore.

    So the control plane owns `controlplane.proto` and we implement it. An
    `Order` is four fields: two nodes, an id and a timestamp. The dependency
    points one way, and it points the right way.

WHY the control plane is not an authority
    It knows no regions and no leaders, deliberately, and this is the module
    that makes that possible. Resolving `pickup_node` to a region to a leader
    can only be done correctly *inside* the fleet at dispatch time — bots
    migrate and leaders rotate, so anything cached outside is wrong by the time
    it is used. The control plane hands an order to any bot it can reach; a
    non-leader forwards to its leader, and a leader forwards to whichever region
    owns the pickup. All of that is `bus.jobs` and none of it is new.

    That is the difference between this and the fleet-wide service this branch
    declined (`docs/boundary.md`): an order source is not a world model.

HOW
    A translation and nothing else. `Order` becomes a `Job`, the existing
    dispatcher answers, and its `JobAck` becomes a `DispatchAck` — the four
    fields line up one for one. Idempotency, forwarding, queueing and retry all
    already live in `bus.jobs`; adding a second implementation of any of them
    here would be adding a second thing to keep right.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

from bus.jobs import Job
from proto import controlplane_pb2, controlplane_pb2_grpc

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)


class ControlPlaneServicer(controlplane_pb2_grpc.ControlPlaneServiceServicer):
    """Served by every bot; routing is the fleet's, not the caller's."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def DispatchOrder(self, request: controlplane_pb2.Order,
                      context: grpc.ServicerContext) -> controlplane_pb2.DispatchAck:
        if not request.order_id:
            # The idempotency key. Without it a retry after a timeout would
            # place the order a second time, and the fleet would send two robots
            # for one box.
            return controlplane_pb2.DispatchAck(
                accepted=False, note="order_id required")

        job = Job(
            job_id=request.order_id,
            pickup_node=request.pickup_node,
            dropoff_node=request.dropoff_node,
        )
        log.info("bot-%d: order %s (%d -> %d) from the control plane",
                 self._bot.bot_id, job.job_id, job.pickup_node, job.dropoff_node)

        ack = self._bot.dispatcher.submit(job)

        reply = controlplane_pb2.DispatchAck(
            accepted=ack.accepted,
            owner_region=ack.owner_region,
            note=ack.note,
        )
        # `optional`, because bot 0 is a real bot: "assigned to bot-0" and "not
        # assigned yet" must not look the same on the wire.
        if ack.HasField("assignee"):
            reply.assignee = ack.assignee
        return reply
