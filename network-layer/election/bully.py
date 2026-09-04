"""Bully leader election — one instance per bot.

WHAT
    The state machine from PROTOCOL.md §5: challenge every higher-priority
    peer with `Elect`; if any answers "I outrank you", stand down and wait for
    its `Coordinator`; otherwise declare victory by sending `Coordinator` to
    everyone. Also the *abdication* primitive (a leader handing off to a
    chosen successor by sending it a Coordinator naming itself).

WHERE
    Owned by `bot.Bot` as `bot.election`. Driven from three places:
      * `bot.on_leader_dead()`      — follower missed T_LEADER_DEAD of acks
      * `election.server`           — incoming Elect / Coordinator RPCs
      * `bot.py` / `bus.migration`  — abdication before migrating or on fault
    Outcomes are reported back to the bot through two callbacks so this
    module never touches roles, senders, or sockets other than its own RPCs.

WHY
    Bully is the simplest algorithm that gives a deterministic winner from a
    cached roster with no central coordinator — exactly the situation after a
    leader dies. Priority (see `election.priority`) already encodes health
    and battery, so "highest priority wins" is also "best candidate wins".

HOW
    * `start_election(peers)` runs `_run_election` on its own thread so a
      gRPC handler can trigger it without blocking.
    * A bot that receives `Elect` from someone it outranks answers ack=True
      AND starts its own election — without that, the challenger stands down
      forever waiting for a Coordinator nobody sends (a bug we shipped once).
    * Unhealthy (`healthy_fn()` false) or departing bots never claim
      leadership: they answer ack=False and refuse to start elections.
    * Every RPC goes through `bus.rpc.pool` (persistent channels) with the
      bot's identity metadata so peers' virtual network admits it.
"""
from __future__ import annotations

import logging
import threading
import time
from enum import Enum, auto
from typing import Callable

import grpc

from config import T_ELECT_TIMEOUT
from proto import fleet_pb2, fleet_pb2_grpc
from peers.table import Peer
from bus.rpc import pool

log = logging.getLogger(__name__)


class Role(Enum):
    LEADER = auto()
    FOLLOWER = auto()


def _outranks(a_priority: int, a_id: int, b_priority: int, b_id: int) -> bool:
    """The one ordering rule used everywhere: priority first, bot_id tiebreak."""
    return (a_priority, a_id) > (b_priority, b_id)


class BullyElection:
    """Election state for one bot. See module docstring for the protocol."""

    def __init__(
        self,
        bot_id: int,
        priority: int,
        address: str,
        on_leader_elected: Callable[[int, str, int], None],
        on_become_leader: Callable[[], None],
        metadata_fn: Callable[[], list[tuple[str, str]]] = lambda: [],
        healthy_fn: Callable[[], bool] = lambda: True,
    ) -> None:
        """
        Args:
            on_leader_elected(leader_id, leader_address, leader_priority): a
                Coordinator arrived. Priority lets the bot detect a *stale*
                Coordinator it should ignore (it outranks the named leader).
            on_become_leader(): we won.
            metadata_fn: identity metadata for outgoing RPCs, evaluated per call
                so region/role stay current across migrations.
            healthy_fn: false → this bot must not lead (PROTOCOL.md §5.1).
        """
        self.bot_id = bot_id
        self.priority = priority  # refreshed each tick by Bot._tick_priority
        self.address = address
        self._on_leader_elected = on_leader_elected
        self._on_become_leader = on_become_leader
        self._metadata_fn = metadata_fn
        self._healthy_fn = healthy_fn

        self._lock = threading.Lock()
        self.election_in_progress = False
        #: Set by the bot when it has begun a graceful shutdown; a departing
        #: bot must never be elected (it is about to disappear).
        self.departing = False
        self._coordinator_event = threading.Event()

    # ---- Eligibility ----------------------------------------------------

    def _eligible(self) -> bool:
        """May this bot claim leadership right now?"""
        return not self.departing and self._healthy_fn()

    # ---- Running an election ---------------------------------------------

    def start_election(self, peers: list[Peer]) -> None:
        """Begin an election on a background thread (no-op if one is running
        or if we are not eligible)."""
        if not self._eligible():
            log.info("bot-%d: not eligible to lead, skipping election", self.bot_id)
            return
        with self._lock:
            if self.election_in_progress:
                return
            self.election_in_progress = True

        log.info("bot-%d starting election (priority=%d)", self.bot_id, self.priority)
        threading.Thread(
            target=self._run_election, args=(peers,), daemon=True, name=f"elect-{self.bot_id}"
        ).start()

    def _run_election(self, peers: list[Peer]) -> None:
        self._coordinator_event.clear()

        higher_peers = [
            p for p in peers if _outranks(p.priority, p.bot_id, self.priority, self.bot_id)
        ]

        any_outranks = False
        for peer in higher_peers:
            try:
                resp = pool.stub(peer.address, fleet_pb2_grpc.ElectionServiceStub).Elect(
                    fleet_pb2.ElectRequest(
                        bot_id=self.bot_id,
                        priority=self.priority,
                        timestamp=int(time.time() * 1000),
                        address=self.address,  # so a winner can Coordinator us back
                    ),
                    timeout=T_ELECT_TIMEOUT,
                    metadata=self._metadata_fn(),
                )
                if resp.ack:
                    any_outranks = True
                    log.info(
                        "bot-%d: peer bot-%d outranks us, standing down",
                        self.bot_id, peer.bot_id,
                    )
                    break
            except grpc.RpcError:
                # Unreachable (dead, partitioned, or its virtual network
                # refused us) — treat as absent and keep going. This is the
                # "cascading failure" path in PROTOCOL.md §4.8.
                log.debug("bot-%d: peer bot-%d unreachable during election", self.bot_id, peer.bot_id)
                continue

        if any_outranks:
            # The higher peer will run its own election (see handle_elect) and
            # send us a Coordinator. If it doesn't within T_ELECT_TIMEOUT it
            # probably died between answering and winning — go again.
            got_coordinator = self._coordinator_event.wait(timeout=T_ELECT_TIMEOUT)
            if not got_coordinator:
                log.warning(
                    "bot-%d: higher peer never declared coordinator, restarting election",
                    self.bot_id,
                )
                with self._lock:
                    self.election_in_progress = False
                self.start_election(peers)
            return

        log.info("bot-%d: won election, declaring coordinator", self.bot_id)
        with self._lock:
            self.election_in_progress = False

        for peer in peers:
            if peer.bot_id == self.bot_id:
                continue
            try:
                pool.stub(peer.address, fleet_pb2_grpc.ElectionServiceStub).Coordinator(
                    fleet_pb2.CoordinatorRequest(
                        bot_id=self.bot_id, priority=self.priority, address=self.address
                    ),
                    timeout=T_ELECT_TIMEOUT,
                    metadata=self._metadata_fn(),
                )
            except grpc.RpcError as e:
                # A peer we could not reach will discover us anyway: its own
                # heartbeats fail, it elects, and either we outrank it (it
                # stands down and we Coordinator it) or the leader-exchange
                # conflict rule sorts it out. Still worth a warning — a
                # *live* peer failing here means it will re-elect needlessly.
                log.warning("bot-%d: could not notify bot-%d of coordinator result: %s %s",
                            self.bot_id, peer.bot_id, e.code(), e.details())

        self._on_become_leader()

    # ---- Incoming RPCs (called from election.server on gRPC threads) ------

    def handle_elect(
        self, candidate_id: int, candidate_priority: int, peers: list[Peer] | None = None
    ) -> bool:
        """A candidate is asking whether we outrank it.

        Returns True ("stand down") only if we outrank it *and* are eligible
        to lead. Crucially, when we say "stand down" we also start our own
        election so that a Coordinator eventually gets sent — the candidate is
        waiting on it.
        """
        if not self._eligible():
            return False

        we_outrank = _outranks(self.priority, self.bot_id, candidate_priority, candidate_id)
        if we_outrank:
            log.info(
                "bot-%d: outrank candidate bot-%d, telling them to stand down",
                self.bot_id, candidate_id,
            )
            if not self.election_in_progress and peers is not None:
                self.start_election(peers)
        return we_outrank

    def handle_coordinator(self, leader_id: int, leader_priority: int, leader_address: str) -> None:
        """Someone declared victory (or is abdicating *to us* — then
        leader_id == our id). Either way the election is over."""
        log.info(
            "bot-%d: received coordinator announcement from bot-%d at %s",
            self.bot_id, leader_id, leader_address,
        )
        with self._lock:
            self.election_in_progress = False
        self._coordinator_event.set()
        self._on_leader_elected(leader_id, leader_address, leader_priority)

    # ---- Abdication ----------------------------------------------------------

    def abdicate(self, successor: Peer) -> bool:
        """Hand leadership to `successor` directly (no election) by sending it
        a Coordinator that names *it* as leader. Used before a leader migrates
        (PROTOCOL.md §4.7) or when a leader becomes unhealthy (§7).

        The caller is responsible for becoming a follower of the successor
        afterwards; other followers find it via `redirect_to` on their next
        heartbeat to us.
        """
        log.info("bot-%d: abdicating to bot-%d at %s", self.bot_id, successor.bot_id, successor.address)
        try:
            pool.stub(successor.address, fleet_pb2_grpc.ElectionServiceStub).Coordinator(
                fleet_pb2.CoordinatorRequest(
                    bot_id=successor.bot_id,
                    priority=successor.priority,
                    address=successor.address,
                ),
                timeout=T_ELECT_TIMEOUT,
                metadata=self._metadata_fn(),
            )
            return True
        except grpc.RpcError:
            log.error("bot-%d: failed to abdicate to bot-%d", self.bot_id, successor.bot_id)
            return False
