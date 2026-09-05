"""Follower ↔ leader heartbeats: the workhorse channel of the protocol.

WHAT
    * `HeartbeatSender` — follower side. Every T_HB, send our state to the
      leader and adopt the roster it returns. Detect a dead leader (missed
      acks), follow redirects, and — when we have no leader at all — probe
      the bootstrap peer list to find one.
    * `RegionServicer` — leader side. Answer heartbeats with the full region
      picture, handle graceful `Departure`, and approve `MigrationRequest`.

WHERE
    Sender thread is started by `Bot.become_follower()` and stopped by
    `Bot.become_leader()`. Servicer is registered on every bot: a follower
    that receives a heartbeat answers with `redirect_to` so stale followers
    find the real leader (PROTOCOL.md §3.1).

WHY
    A single request/response per second carries everything a bot needs to
    operate without the leader: the roster with addresses (for elections)
    and the map of other regions' leaders (for migration). PROTOCOL.md §7:
    the leader must never be the only holder of any state — this is how
    that copy reaches every bot.

HOW
    * Leader-side roster is `upsert`ed per heartbeat and evicted by the run
      loop after T_DEAD. Follower-side roster is `replace`d wholesale per ack.
    * `MIGRATING_OUT` is NOT stored on the Peer record (a heartbeat would
      overwrite it within T_HB); it lives in `bot.migrating_out` and is
      overlaid onto the record's `state` when the ack is serialized.
    * `MigrationRequest` never blocks on the destination: it records the
      migration, replies immediately, and does the leader→leader handoff on
      a background thread (PROTOCOL.md §4.6). The migrating bot retries its
      join until the handoff lands.
    * All RPCs use `bus.rpc.pool` (persistent channels) and carry identity
      metadata for the receiver's virtual network.
"""
from __future__ import annotations

import config

import logging
import threading
import time
from typing import TYPE_CHECKING

import grpc

from config import T_HB, T_DEAD, T_MIGRATION_TIMEOUT
from proto import fleet_pb2, fleet_pb2_grpc
from peers.table import Peer, Leader
from bus.rpc import pool
from bus.jobs import Job
from election.bully import Role

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)

#: Missed acks before we declare the leader dead (T_LEADER_DEAD in heartbeats).
MAX_MISSED_ACKS = max(1, round(T_DEAD / T_HB))


def _peer_from_record(p: fleet_pb2.PeerRecord) -> Peer:
    return Peer(
        bot_id=p.bot_id, address=p.address, priority=p.priority,
        state=p.state, battery=p.battery, latest_node_id=p.latest_node_id,
        node_trail=list(p.node_trail), mission=p.mission, fault=p.fault,
        job_id=p.job_id, cargo_state=p.cargo_state,
    )


def _leader_from_record(ld: fleet_pb2.LeaderRecord) -> Leader:
    return Leader(region_id=ld.region_id, bot_id=ld.bot_id, address=ld.address)


class HeartbeatSender:
    """Follower-side loop. See module docstring."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.stop()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._loop, args=(self._stop,), daemon=True, name=f"hb-{self._bot.bot_id}"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        # A redirect handled inside the loop calls become_follower → start()
        # → stop() on this very thread; joining ourselves would raise.
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=config.T_THREAD_JOIN)
        self._thread = None

    # ---- Loop ---------------------------------------------------------------

    def _loop(self, stop: threading.Event) -> None:
        # Capture our own stop event: start() may be called again (redirect
        # path) while this iteration is still finishing, and we must exit on
        # the old event rather than pick up the new one and run twice.
        missed_acks = 0
        while not stop.is_set():
            leadership = self._bot.leadership()
            if leadership.leader_address is None:
                # No leader known: unhealthy at boot, or rejoining after our
                # cached leader vanished. Ask around (PROTOCOL.md §4.3, §6).
                self._probe_for_leader()
                stop.wait(T_HB)
                continue

            try:
                ack = self._send(leadership.leader_address)
            except grpc.RpcError as e:
                missed_acks += 1
                log.warning(
                    "bot-%d: missed ack from leader (%d/%d): %s",
                    self._bot.bot_id, missed_acks, MAX_MISSED_ACKS, e.code(),
                )
                if missed_acks >= MAX_MISSED_ACKS:
                    log.warning("bot-%d: leader unreachable, triggering election", self._bot.bot_id)
                    self._bot.on_leader_dead()
                    missed_acks = 0
                stop.wait(T_HB)
                continue

            missed_acks = 0
            self._bot.last_ack_at = time.monotonic()

            if ack.redirect_to:
                # The bot we heartbeated is not the leader (any more). Follow
                # the pointer, but keep this loop and its 1 Hz pace: a redirect
                # chain that comes back to us makes us leader (retarget handles
                # that), and a ping-pong must never run at RPC speed.
                log.info("bot-%d: redirected to leader bot-%d at %s",
                         self._bot.bot_id, ack.leader_bot_id, ack.redirect_to)
                self._bot.retarget(ack.leader_bot_id, ack.redirect_to)
                if self._bot.role == Role.LEADER:
                    return  # retarget promoted us; become_leader stopped this sender
                stop.wait(T_HB)
                continue

            self._bot.peer_table.replace(
                [_peer_from_record(p) for p in ack.region_peers],
                [_leader_from_record(ld) for ld in ack.other_leaders],
            )
            self._bot.on_ack_jobs([Job.from_proto(j) for j in ack.jobs])
            stop.wait(T_HB)

    def _send(self, leader_addr: str) -> fleet_pb2.HeartbeatAck:
        return pool.stub(leader_addr, fleet_pb2_grpc.RegionServiceStub).Heartbeat(
            self._bot.heartbeat_payload(),
            timeout=T_HB * 2,
            metadata=self._bot.rpc_metadata(),
        )

    def _probe_for_leader(self) -> None:
        """Heartbeat each bootstrap peer until one tells us who leads.

        A leader answers with a roster (and its own id); a follower answers
        with `redirect_to`; a bot in another region refuses us (its virtual
        network) and we move on. Whoever answers first wins — subsequent
        acks/redirects converge us on the real leader.
        """
        for addr in self._bot.peer_leaders:
            if addr == self._bot.address:
                continue
            try:
                ack = self._send(addr)
            except grpc.RpcError:
                continue
            target = ack.redirect_to or addr
            if ack.redirect_to == "" and ack.leader_bot_id == 0 and not ack.region_peers:
                # A follower that doesn't know a leader either. Keep looking.
                continue
            log.info("bot-%d: discovered leader bot-%d at %s via %s",
                     self._bot.bot_id, ack.leader_bot_id, target, addr)
            self._bot.retarget(ack.leader_bot_id, target)
            return


class RegionServicer(fleet_pb2_grpc.RegionServiceServicer):
    """Leader-side handlers (and the follower-side redirect). See module docstring."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    # ---- Heartbeat ---------------------------------------------------------

    def Heartbeat(self, request: fleet_pb2.HeartbeatRequest, context: grpc.ServicerContext):
        leadership = self._bot.leadership()
        if leadership.role != Role.LEADER:
            # We are a follower. Point the caller at whoever we follow (may be
            # empty if we are still discovering — caller keeps probing).
            return fleet_pb2.HeartbeatAck(
                redirect_to=leadership.leader_address or "",
                leader_bot_id=leadership.leader_id or 0,
            )

        # The virtual network already rejects cross-region callers; this is a
        # belt-and-braces check on the body in case policy is ever disabled.
        if request.region_id != self._bot.region_id:
            log.warning("bot-%d: rejecting heartbeat from bot-%d (region %d != %d)",
                        self._bot.bot_id, request.bot_id, request.region_id, self._bot.region_id)
            return fleet_pb2.HeartbeatAck()

        prev = self._bot.peer_table.get(request.bot_id)
        cur = Peer(
            bot_id=request.bot_id, address=request.address, priority=request.priority,
            state=request.state, battery=request.battery, latest_node_id=request.latest_node_id,
            node_trail=list(request.node_trail), mission=request.mission, fault=request.fault,
            job_id=request.job_id, cargo_state=request.cargo_state,
        )
        self._bot.peer_table.upsert(cur)
        # Jobs are tracked by *watching* heartbeats, never by extra messages.
        self._bot.dispatcher.observe(prev, cur)
        return self._bot.roster_ack()

    # ---- Departure ---------------------------------------------------------

    def Departure(self, request: fleet_pb2.DepartureRequest, context: grpc.ServicerContext):
        """Graceful leave, or the final step of a migration out of here.
        Clears both the roster entry and any migrating_out record so the
        source-side migration timeout has nothing left to expire."""
        log.info("bot-%d: received departure from bot-%d", self._bot.bot_id, request.bot_id)
        self._bot.peer_table.remove(request.bot_id)
        self._bot.migrating_out.pop(request.bot_id)
        return fleet_pb2.DepartureAck()

    # ---- MigrationRequest ---------------------------------------------------

    def MigrationRequest(self, request: fleet_pb2.MigrationReq, context: grpc.ServicerContext):
        """Approve a follower's move to another region (PROTOCOL.md §4.6 step 2).

        Policy today: always approve (the fleet layer may add "has active job"
        / "region too small" later). We record the bot in `migrating_out`,
        reply immediately, and perform the leader→leader handoff (step 3) on a
        background thread so no gRPC worker blocks on a cross-region call.
        """
        peer = self._bot.peer_table.get(request.bot_id)
        if peer is None:
            log.warning("bot-%d: migration denied for unknown bot-%d", self._bot.bot_id, request.bot_id)
            return fleet_pb2.MigrationReqAck(approved=False)

        dest = self._bot.peer_table.get_leader(request.destination_region_id)
        self._bot.migrating_out.mark(request.bot_id, request.destination_region_id)

        if dest is None:
            # Empty destination (PROTOCOL.md §4.2): approve with no leader so
            # the bot self-declares there. Nobody to hand off to.
            log.info("bot-%d: approved migration of bot-%d to empty region %d (solo)",
                     self._bot.bot_id, request.bot_id, request.destination_region_id)
            return fleet_pb2.MigrationReqAck(approved=True)

        threading.Thread(
            target=self._handoff, args=(request.bot_id, peer, dest), daemon=True,
            name=f"handoff-{request.bot_id}",
        ).start()

        log.info("bot-%d: approved migration of bot-%d to region %d (handoff in progress)",
                 self._bot.bot_id, request.bot_id, request.destination_region_id)
        return fleet_pb2.MigrationReqAck(
            approved=True,
            destination_leader=fleet_pb2.LeaderRecord(
                region_id=dest.region_id, bot_id=dest.bot_id, address=dest.address
            ),
        )

    def _handoff(self, bot_id: int, peer: Peer, dest: Leader) -> None:
        """Leader → leader: tell the destination to expect `bot_id`.

        On failure we drop our migrating_out record: the bot's join will be
        refused by the destination's virtual network, its migrator will back
        off and retry the whole request later — by which time we may know a
        fresher destination leader.
        """
        try:
            ack = pool.stub(dest.address, fleet_pb2_grpc.LeaderExchangeServiceStub).MigrationHandoff(
                fleet_pb2.MigrationHandoffReq(
                    bot_id=bot_id, source_region_id=self._bot.region_id,
                    bot_priority=peer.priority, bot_address=peer.address,
                ),
                timeout=T_MIGRATION_TIMEOUT,
                metadata=self._bot.rpc_metadata(),
                wait_for_ready=True,
            )
        except grpc.RpcError as e:
            log.warning("bot-%d: handoff of bot-%d to region %d failed: %s %s",
                        self._bot.bot_id, bot_id, dest.region_id, e.code(), e.details())
            self._bot.migrating_out.pop(bot_id)
            return
        if not ack.accepted:
            log.warning("bot-%d: region %d refused handoff of bot-%d",
                        self._bot.bot_id, dest.region_id, bot_id)
            self._bot.migrating_out.pop(bot_id)
