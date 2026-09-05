"""AdminService — look inside a running bot, and feed it robot state.

WHAT
    * `GetState`         — everything a test or an operator wants to assert
                           on: role, leader, region, roster, jobs, own job.
    * `InjectRobotState` — push a `RobotState` into the bot's `RobotSource`
                           exactly as the real robot bridge would (a QR scan
                           in another region, a fault, cargo progress …).

WHERE
    Registered on every bot's server, but the virtual network (`bus/policy.py`)
    only admits it when `config.ADMIN_ENABLED` is true. `up.py` enables it
    for local Docker fleets; leave it off in production.

WHY
    The Docker test harness (`tests/test_docker.py`) needs to drive and
    inspect bots that live in containers. Parsing logs is brittle; a typed
    RPC is not. The same surface is handy for a fleet dashboard later.

HOW
    Thin adapter over `bot.Bot`. `InjectRobotState` requires the default
    `QueueRobotSource`; with a real bridge plugged in it is refused, because
    two sources of truth about the robot is exactly the thing we do not want.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

from proto import fleet_pb2, fleet_pb2_grpc

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)


class AdminServicer(fleet_pb2_grpc.AdminServiceServicer):
    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def GetState(self, request: fleet_pb2.Empty, context: grpc.ServicerContext):
        b = self._bot
        ls = b.leadership()
        roster = b.roster_ack()
        return fleet_pb2.BotState(
            bot_id=b.bot_id, region_id=b.region_id, role=ls.role.name.lower(),
            leader_bot_id=ls.leader_id or 0, leader_address=ls.leader_address or "",
            priority=b.priority, state=b.effective_state(),
            current_job_id=b.current_job.job_id if b.current_job else "", cargo_state=b.cargo_state,
            roster=roster.region_peers, other_leaders=roster.other_leaders, jobs=roster.jobs,
            desired_region_id=b.desired_region_id or 0, leader_settled=b.leader_settled(),
            reservations=_held_claims(b),
        )

    def InjectRobotState(self, request: fleet_pb2.RobotStateMsg, context: grpc.ServicerContext):
        src = self._bot._robot_source
        # Duck-typed on purpose: in a container `bot.py` runs as __main__, so
        # `from bot import QueueRobotSource` would be a *second* copy of the
        # class and isinstance() would always be False.
        push = getattr(src, "push", None)
        if push is None:
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, "a real robot bridge is attached")
        RobotState = type(self._bot).__module__ and __import__(type(self._bot).__module__).RobotState
        push(RobotState(
            latest_node_id=request.latest_node_id, region_id=request.region_id, battery=request.battery,
            state=request.state or "IDLE", mission=request.mission or "IDLE", fault=request.fault,
            job_id=request.job_id, cargo_state=request.cargo_state,
        ))
        log.info("bot-%d: admin injected robot state (region=%d node=%d state=%s mission=%s/%s)",
                 self._bot.bot_id, request.region_id, request.latest_node_id, request.state,
                 request.mission, request.cargo_state)
        return fleet_pb2.Empty()


def _held_claims(bot) -> list[fleet_pb2.HeldClaim]:
    """This bot's ledger, flattened for inspection (PROTOCOL.md §15).

    Own claims and neighbours' in one list, told apart by `bot_id`. Without this
    a container test can start two bots and watch them announce, but has no way
    to see whether a claim ever arrived.
    """
    ledger = bot.ledger
    claims = list(ledger.mine) + list(ledger.peer_claims())
    return [
        fleet_pb2.HeldClaim(
            bot_id=c.bot_id, node_id=c.node_id, start_ms=c.start_ms, end_ms=c.end_ms
        )
        for c in claims
    ]
