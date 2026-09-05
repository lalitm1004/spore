"""Migration — moving a bot from one region's roster to another's.

WHAT
    * `Migrator` — the bot-side state machine. Owns the whole handshake
      (PROTOCOL.md §4.6), retries with backoff, and *reconciles*: as long as
      the region the robot is physically in (`bot.desired_region_id`, from
      the last QR scan) differs from the region we are a member of
      (`bot.region_id`), it keeps trying.
    * `MigrationJoinServicer` — destination-leader side of step 4.

WHERE
    `bot.Bot` owns one `Migrator` and calls `migrator.tick()` every run-loop
    tick. The servicer is registered on every bot (any bot may become a
    leader). The *source*-leader half of the handshake (approve + handoff)
    lives in `bus.heartbeat.RegionServicer`.

WHY
    The first version ran the handshake once, on the tick a QR update
    arrived, and gave up on any failure. A bot could end up physically in
    region 2 while still a member of region 14 — forever, if it then parked
    and stopped emitting updates. Migration must be a *state* with its own
    loop: idempotent, retried, and driven by stored desired-vs-actual, not
    by the arrival of an event.

HOW
    Phases (see `Phase`):

        IDLE ─┬─▶ REQUESTING ─▶ JOINING ─▶ DEPARTING ─▶ IDLE   (success)
              │        │            │
              └────────┴────────────┴──▶ FAILED ─(backoff)─▶ IDLE

    * REQUESTING: if we lead, abdicate to the best peer first (§4.7). Then
      `MigrationRequest` to our leader. Approval carries the destination
      leader (or nothing — empty region, we go solo).
    * JOINING: `MigrationJoin` to the destination, retried every 0.5 s until
      T_MIGRATION_TIMEOUT — the source leader's handoff runs asynchronously
      and may land a moment after we first try.
    * DEPARTING: `Departure` to the old leader *before* switching identity
      (so it carries the old region_id and passes the old leader's virtual
      network), then adopt the new roster and leader.
    * FAILED: exponential backoff (1, 2, 4 … capped at T_MIGRATION_BACKOFF_MAX)
      then back to IDLE; `tick()` will restart if desired ≠ actual.
    * A migration only *starts* once a leader is settled (`bot.leader_settled()`),
      so a bot never leaves mid-election and orphans a region.

    While in flight, `bot.effective_state()` reports "MIGRATING" so the old
    leader (via heartbeats) and the whole region (via acks) can see it.
"""
from __future__ import annotations

import config

import logging
import threading
import time
from enum import Enum, auto
from typing import TYPE_CHECKING

import grpc

from config import T_MIGRATION_TIMEOUT, T_MIGRATION_BACKOFF_MAX, T_HB
from proto import fleet_pb2, fleet_pb2_grpc
from peers.table import Peer, Leader
from bus.rpc import pool

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)




class Phase(Enum):
    IDLE = auto()
    REQUESTING = auto()
    JOINING = auto()
    DEPARTING = auto()
    FAILED = auto()


class Migrator:
    """Bot-side migration state machine. See module docstring."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot
        self._lock = threading.Lock()
        self.phase = Phase.IDLE
        self.target_region: int | None = None
        self.attempts = 0
        self._next_attempt_at = 0.0
        self._thread: threading.Thread | None = None

    # ---- Public ---------------------------------------------------------------

    @property
    def in_flight(self) -> bool:
        return self.phase in (Phase.REQUESTING, Phase.JOINING, Phase.DEPARTING)

    def tick(self) -> None:
        """Called every run-loop tick. Starts (or restarts) a migration when the
        robot's physical region differs from our membership and nothing is in
        flight. This is the reconciliation loop."""
        desired = self._bot.desired_region_id
        if desired is None or desired == self._bot.region_id:
            if self.phase == Phase.FAILED:
                self.phase = Phase.IDLE  # target reached by other means, or QR changed back
            return
        if self.in_flight:
            return
        now = time.monotonic()
        if now < self._next_attempt_at:
            return
        if not self._bot.leader_settled():
            # PROTOCOL.md §5.7: never migrate mid-election.
            log.debug("bot-%d: want region %d but leader not settled; waiting",
                      self._bot.bot_id, desired)
            return

        with self._lock:
            if self.in_flight:
                return
            self.phase = Phase.REQUESTING
            self.target_region = desired
        self._thread = threading.Thread(
            target=self._run, args=(desired,), daemon=True, name=f"migrate-{self._bot.bot_id}"
        )
        self._thread.start()

    # ---- The handshake --------------------------------------------------------

    def _run(self, target: int) -> None:
        bot = self._bot
        log.info("bot-%d: migration attempt %d: region %d -> %d",
                 bot.bot_id, self.attempts + 1, bot.region_id, target)
        try:
            ok = self._attempt(target)
        except Exception:  # never let a bug here kill the loop silently
            log.exception("bot-%d: migration crashed", bot.bot_id)
            ok = False

        if ok:
            self.attempts = 0
            self.phase = Phase.IDLE
            log.info("bot-%d: migration complete, now in region %d", bot.bot_id, bot.region_id)
        else:
            self.attempts += 1
            delay = min(T_MIGRATION_BACKOFF_MAX, 2.0 ** (self.attempts - 1))
            self._next_attempt_at = time.monotonic() + delay
            self.phase = Phase.FAILED
            log.warning("bot-%d: migration to region %d failed (attempt %d); retry in %.1fs",
                        bot.bot_id, target, self.attempts, delay)

    def _attempt(self, target: int) -> bool:
        from election.bully import Role

        bot = self._bot
        deadline = time.monotonic() + T_MIGRATION_TIMEOUT

        # --- REQUESTING: abdicate if leading, then ask our leader ------------
        self.phase = Phase.REQUESTING
        if bot.role == Role.LEADER:
            # Hand off to a healthy, preferably *free* peer (§5.6 succession):
            # a busy successor would drive away and have to hand off again.
            successor = bot.peer_table.best_successor(exclude=bot.bot_id)
            if successor is not None:
                if not bot.election.abdicate(successor):
                    return False
                bot.become_follower(successor.bot_id, successor.address)
            # else: we are alone here; nobody to hand off to (§4.7).

        dest_addr: str | None
        dest_id: int | None
        if bot.role == Role.FOLLOWER and bot.leader_address:
            try:
                ack = pool.stub(bot.leader_address, fleet_pb2_grpc.RegionServiceStub).MigrationRequest(
                    fleet_pb2.MigrationReq(
                        bot_id=bot.bot_id, destination_region_id=target,
                        timestamp=int(time.time() * 1000),
                    ),
                    timeout=T_MIGRATION_TIMEOUT,
                    metadata=bot.rpc_metadata(),
                    wait_for_ready=True,
                )
            except grpc.RpcError as e:
                log.warning("bot-%d: migration request failed: %s %s", bot.bot_id, e.code(), e.details())
                return False
            if not ack.approved:
                log.warning("bot-%d: migration denied by leader", bot.bot_id)
                return False
            dest_addr = ack.destination_leader.address or None
            dest_id = ack.destination_leader.bot_id if dest_addr else None
        else:
            # Solo leader with nobody to ask: use our own map of leaders.
            dest = bot.peer_table.get_leader(target)
            dest_addr = dest.address if dest else None
            dest_id = dest.bot_id if dest else None

        if dest_addr is None:
            return self._go_solo(target)

        # --- JOINING: retry until the async handoff has landed ----------------
        self.phase = Phase.JOINING
        join_ack = None
        while time.monotonic() < deadline:
            try:
                join_ack = pool.stub(dest_addr, fleet_pb2_grpc.MigrationJoinServiceStub).MigrationJoin(
                    fleet_pb2.MigrationJoinReq(
                        bot_id=bot.bot_id, source_region_id=bot.region_id, priority=bot.priority,
                        address=bot.address, battery=bot.battery, state=bot.state,
                        latest_node_id=bot.latest_node_id, node_trail=list(bot.node_trail),
                    ),
                    timeout=max(0.5, deadline - time.monotonic()),
                    metadata=bot.rpc_metadata(),
                    wait_for_ready=True,
                )
                if join_ack.accepted:
                    break
                log.debug("bot-%d: join not accepted yet, retrying", bot.bot_id)
            except grpc.RpcError as e:
                if e.code() != grpc.StatusCode.PERMISSION_DENIED:
                    log.warning("bot-%d: migration join failed: %s %s", bot.bot_id, e.code(), e.details())
                    return False
                # PERMISSION_DENIED == "no handoff for you yet". Wait for it.
            time.sleep(config.T_JOIN_RETRY)

        if join_ack is None or not join_ack.accepted:
            log.warning("bot-%d: destination never accepted join within %.0fs", bot.bot_id, T_MIGRATION_TIMEOUT)
            return False

        # --- DEPARTING: leave the old region, then switch identity -------------
        self.phase = Phase.DEPARTING
        _send_departure(bot, bot.leader_address)  # still carrying the old region_id
        bot.adopt_region(
            region_id=target,
            leader_id=dest_id,
            leader_address=dest_addr,
            peers=[Peer(bot_id=p.bot_id, address=p.address, priority=p.priority, state=p.state,
                        battery=p.battery, latest_node_id=p.latest_node_id, node_trail=list(p.node_trail))
                   for p in join_ack.region_peers],
            leaders=[Leader(region_id=ld.region_id, bot_id=ld.bot_id, address=ld.address)
                     for ld in join_ack.other_leaders],
        )
        return True

    def _go_solo(self, target: int) -> bool:
        """Nobody leads the destination: depart, then self-declare there (§4.2)."""
        bot = self._bot
        log.info("bot-%d: no leader in region %d, self-declaring", bot.bot_id, target)
        self.phase = Phase.DEPARTING
        _send_departure(bot, bot.leader_address)
        bot.adopt_region(region_id=target, leader_id=None, leader_address=None, peers=[], leaders=[])
        bot.become_leader()
        return True


def _send_departure(bot: Bot, leader_addr: str | None) -> None:
    """Best effort: tell the old leader we are gone so it need not wait T_DEAD."""
    if not leader_addr:
        return
    try:
        pool.stub(leader_addr, fleet_pb2_grpc.RegionServiceStub).Departure(
            fleet_pb2.DepartureRequest(bot_id=bot.bot_id, timestamp=int(time.time() * 1000)),
            timeout=T_HB * 2,
            metadata=bot.rpc_metadata(),
            wait_for_ready=True,
        )
    except grpc.RpcError as e:
        # Old leader gone or changed: it (or its successor) evicts us after T_DEAD.
        log.debug("bot-%d: couldn't send departure to old leader: %s", bot.bot_id, e.code())


class MigrationJoinServicer(fleet_pb2_grpc.MigrationJoinServiceServicer):
    """Destination-leader side of step 4. The virtual network only admits
    callers present in `bot.pending_incoming`, so by the time we run the
    handoff has already happened; the check here is defence in depth."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def MigrationJoin(self, request: fleet_pb2.MigrationJoinReq, context: grpc.ServicerContext):
        from election.bully import Role

        if self._bot.role != Role.LEADER:
            return fleet_pb2.MigrationJoinAck(accepted=False)
        if request.bot_id not in self._bot.pending_incoming:
            log.warning("bot-%d: unexpected migration join from bot-%d (no handoff)",
                        self._bot.bot_id, request.bot_id)
            return fleet_pb2.MigrationJoinAck(accepted=False)

        self._bot.pending_incoming.pop(request.bot_id)
        self._bot.peer_table.upsert(
            Peer(bot_id=request.bot_id, address=request.address, priority=request.priority,
                 state=request.state, battery=request.battery, latest_node_id=request.latest_node_id,
                 node_trail=list(request.node_trail))
        )
        log.info("bot-%d: bot-%d joined region %d via migration",
                 self._bot.bot_id, request.bot_id, self._bot.region_id)

        roster = self._bot.roster_ack()
        return fleet_pb2.MigrationJoinAck(
            accepted=True, region_peers=roster.region_peers, other_leaders=roster.other_leaders
        )
