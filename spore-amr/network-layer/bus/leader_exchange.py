"""Leader ↔ leader heartbeats: cross-region awareness, bootstrap discovery,
and split-brain resolution.

WHAT
    * `LeaderExchangeSender` — while we are a leader, heartbeat every other
      leader we know of (plus the bootstrap PEER_LEADERS list) every
      T_LEADER_HB.
    * `LeaderExchangeServicer` — answer those heartbeats, and accept
      `MigrationHandoff` from other leaders.

WHERE
    Sender runs on its own thread, started by `Bot.become_leader()` and
    stopped by `Bot.become_follower()`. Servicer is registered on every bot
    (followers answer too — see "conflict" below).

WHY
    This one channel does three jobs (PROTOCOL.md §4.1, §5.4):
      1. Discovery. Every bot boots as leader of its region and heartbeats
         everyone in PEER_LEADERS. That is how bots find each other with no
         registry and no designated first leader.
      2. Conflict → election. If two bots both claim the same region, the
         lower-priority one yields on the spot. At bootstrap this collapses
         N self-declared leaders down to one within a heartbeat round; after
         a network partition heals it resolves the split brain.
      3. Cross-region map. Leaders of *different* regions learn each other's
         addresses, which followers then receive in `other_leaders` and use
         to start migrations.

HOW
    The conflict check runs on BOTH sides of a heartbeat — the receiver sees
    it in the request, the sender sees it in the ack — so it resolves in one
    round trip regardless of who heartbeats first. `Bot.on_same_region_leader_
    conflict` applies the priority rule.

    Note for maintainers: the sender-side check calls `become_follower()`
    from *inside the sender's own thread*, which stops the sender. The stop
    path must therefore never join the current thread, and the loop captures
    its own stop event so a restarted sender cannot resurrect a stale loop.
"""
from __future__ import annotations

import config

import logging
import threading
import time
from typing import TYPE_CHECKING

import grpc

from config import T_LEADER_HB
from proto import fleet_pb2, fleet_pb2_grpc
from peers.table import Leader
from bus.rpc import pool

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)


class LeaderExchangeSender:
    """Background loop: heartbeat all known/bootstrap leaders every T_LEADER_HB."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.stop()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._loop, args=(self._stop,), daemon=True, name=f"lx-{self._bot.bot_id}"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        # May be called from inside the loop thread (conflict on our own ack
        # path → become_follower → stop). Joining ourselves raises.
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=config.T_THREAD_JOIN)
        self._thread = None

    def _loop(self, stop: threading.Event) -> None:
        # `stop` is captured per-thread: if start() is called again while we
        # are mid-iteration, we must exit on *our* event, not the new one.
        while not stop.is_set():
            for addr in self._get_targets():
                if stop.is_set():
                    break
                self._send_heartbeat(addr)
            stop.wait(T_LEADER_HB)

    def _get_targets(self) -> list[str]:
        """Every other-region leader we know, plus the bootstrap list."""
        addrs: set[str] = {ld.address for ld in self._bot.peer_table.all_leaders()}
        addrs.update(a for a in self._bot.peer_leaders if a != self._bot.address)
        return list(addrs)

    def _send_heartbeat(self, addr: str) -> None:
        try:
            ack = pool.stub(addr, fleet_pb2_grpc.LeaderExchangeServiceStub).LeaderHeartbeat(
                self._bot.leader_hb_payload(),
                timeout=T_LEADER_HB * 2,
                metadata=self._bot.rpc_metadata(),
            )
        except grpc.RpcError:
            # Dead peer, a follower that no longer serves this (policy denies
            # non-leaders nothing here — followers *do* answer), or a bot in
            # a region we haven't met. All fine; try again next round.
            log.debug("bot-%d: leader exchange to %s failed", self._bot.bot_id, addr)
            return

        if ack.region_id == self._bot.region_id and ack.leader_bot_id != self._bot.bot_id:
            # Same region, different bot claiming leadership → conflict.
            self._bot.on_same_region_leader_conflict(ack.leader_bot_id, ack.address, ack.priority)
        elif ack.region_id != self._bot.region_id:
            _learn_region(self._bot, ack)


def _learn_region(bot: Bot, msg) -> None:
    """Record another region's leader and where its bots have been lately."""
    bot.peer_table.upsert_leader(
        Leader(region_id=msg.region_id, bot_id=msg.leader_bot_id, address=msg.address)
    )
    bot.peer_table.set_region_locations(
        msg.region_id, {loc.bot_id: list(loc.node_trail) for loc in msg.locations}
    )


class LeaderExchangeServicer(fleet_pb2_grpc.LeaderExchangeServiceServicer):
    """Answer leader heartbeats; accept migration handoffs (destination side)."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def LeaderHeartbeat(self, request: fleet_pb2.LeaderHBRequest, context: grpc.ServicerContext):
        if request.region_id == self._bot.region_id and request.leader_bot_id != self._bot.bot_id:
            self._bot.on_same_region_leader_conflict(
                request.leader_bot_id, request.address, request.priority
            )
        elif request.region_id != self._bot.region_id:
            _learn_region(self._bot, request)
        # Mirror our own summary back so both sides learn in one round trip.
        return self._bot.leader_hb_payload(ack=True)

    def MigrationHandoff(self, request: fleet_pb2.MigrationHandoffReq, context: grpc.ServicerContext):
        """Source leader says "expect bot X". Record it so the bot's
        MigrationJoin passes the virtual network; expires after
        T_MIGRATION_TIMEOUT if the bot never shows (PROTOCOL.md §8)."""
        from election.bully import Role

        if self._bot.role != Role.LEADER:
            # We are not leading this region any more; the source leader's
            # record of us is stale. Refusing makes it deny the migration and
            # the bot will retry after learning the real leader.
            return fleet_pb2.MigrationHandoffAck(accepted=False)

        log.info(
            "bot-%d: accepting migration handoff for bot-%d from region %d",
            self._bot.bot_id, request.bot_id, request.source_region_id,
        )
        self._bot.pending_incoming.mark(request.bot_id, request.source_region_id)
        return fleet_pb2.MigrationHandoffAck(accepted=True)

    # ---- Jobs (PROTOCOL.md §14) ------------------------------------------

    def ForwardJob(self, request: fleet_pb2.ForwardJobReq, context: grpc.ServicerContext):
        """Another region had nobody free. Try here; never keep it if we can't."""
        return self._bot.dispatcher.handle_forward(request)

    def JobEvent(self, request: fleet_pb2.Job, context: grpc.ServicerContext):
        """A leader saw one of *our* jobs progress or fail in its region.
        `owned` tells it whether it can stop re-sending."""
        owned = self._bot.dispatcher.handle_event(request)
        return fleet_pb2.JobEventAck(owned=owned)
