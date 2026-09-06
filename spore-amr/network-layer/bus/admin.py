"""AdminService — look inside a running bot.

WHAT
    * `GetState` — everything a test or an operator wants to assert on: role,
      leader, region, roster, jobs, own job, and the claims it holds.

    Read-only, and that is new. It used to carry `InjectRobotState` and
    `InjectObstruction`, which pushed a whole robot snapshot or a blockage
    straight into the bot, around the QR read, the companion and the wire. They
    were the last back doors, and they cost more than they looked: the container
    suite could not see that production never fed position at all, because
    injection supplied by hand the one thing nothing else supplied. Both are
    gone. A robot reports over `RobotNetwork.Session` like a robot.

WHERE
    Registered on every bot's server, but the virtual network (`bus/policy.py`)
    only admits it when `config.ADMIN_ENABLED` is true. `up.py` enables it
    for local Docker fleets; leave it off in production.

WHY
    The container tier (`tests/containers/`) needs to inspect
    inspect bots that live in containers. Parsing logs is brittle; a typed
    RPC is not. The same surface is handy for a fleet dashboard later.

HOW
    Thin read-only adapter over `bot.Bot`.
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
