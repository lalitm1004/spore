"""The announce step: tell the bots nearby what we are holding.

WHAT
    `ReservationSender.tick()` — expire neighbours who have gone quiet, give way
    where we lost a contest, claim the node we are on, and announce it to the
    bots close enough to care.

WHERE
    Called from `bot.Bot._run_loop()` once per `T_ANNOUNCE` (which defaults to
    `T_HB`), after the robot's state has been read so it sees this tick's
    position. Not a thread of its own — announcing is cheap and there is nothing
    to wait for.

WHY
    This is the whole point of PROTOCOL.md §15: reservations go bot to bot, so
    they still work when there is no leader. The roster the leader hands out is
    used only to *find* neighbours; once we have an address we talk to it
    directly, and a leader dying does not interrupt that.

HOW
    * Rank comes from `election.priority.yield_priority` — the same call
      `bot.self_peer()` makes when serialising a `PeerRecord`, so the number a
      neighbour sees in an announcement matches the one in the roster.
    * The claim is a rolling window on the node the robot is standing on. It is
      re-proposed every tick and the ledger treats that as carrying on holding
      the node rather than as a new claim each time.
    * Unreachable peers are logged at debug and nothing else. A missed
      announcement is not an error: the claim is repeated next tick, and if we
      stay silent long enough our claims lapse on the neighbour's side by design.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

import config
from bus.rpc import pool
from election import priority as prio
from proto import fleet_pb2_grpc
from reservations import now_ms
from reservations.claims import claim_window_ms
from reservations.server import to_proto
from reservations.vicinity import in_claim_range

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)

CS_CARRYING = ("EN_ROUTE", "DROPOFF")


class ReservationSender:
    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def tick(self) -> None:
        bot = self._bot
        ledger = bot.ledger
        now = now_ms()

        for bot_id in ledger.expire(now):
            log.debug("bot-%d: reservations from bot-%d lapsed", bot.bot_id, bot_id)

        # Both sides of a contest reach the same verdict; only the loser acts on
        # it, which is how the node frees up without either asking the other.
        for contest in ledger.lost():
            log.info(
                "bot-%d: giving way on node %d to bot-%d",
                bot.bot_id, contest.node_id, contest.theirs.bot_id,
            )
            ledger.withdraw()
            break

        self._claim(now)
        self._announce(now)

    # ---- Our claim ----------------------------------------------------------

    def _claim(self, now: int) -> None:
        """Hold the node the robot is standing on.

        The smallest honest claim there is, and real information: a neighbour
        planning through here needs to know somebody is sitting on it. Anything
        richer needs a path planner deciding where this bot is going next.
        """
        bot = self._bot
        if not bot.latest_node_id:
            bot.ledger.withdraw()   # no QR scan yet: we cannot say where we are
            return
        bot.ledger.propose(
            [(bot.latest_node_id, now, now + self._hold_ms())], now, rank=self._rank()
        )

    def _hold_ms(self) -> int:
        """How long to hold the node the robot is standing on.

        Two quantities have to be covered and they answer different questions:

          **staying announced** -- two announce periods, so the claim outlives
          the gap between one announcement and the next;
          **staying honest** -- one traversal plus the planner's safety margin,
          so a neighbour that decides *right now* to drive in here still finds
          the node taken when it arrives.

        The claim takes the larger. Tying it to the announce period alone was
        very nearly a collision: at production timings `2 * T_ANNOUNCE` and one
        hop are both exactly 2000 ms, so a stationary robot's claim expired on
        the same millisecond a neighbour reached it -- and because overlap is
        strict, that read as *free*. Nothing about a 200 cm node spacing and a
        1 s heartbeat makes that coincidence a design; move either constant a
        little and a robot standing still becomes invisible to the robot
        driving at it.

        With no map loaded there is no traversal to reason about and nothing
        can plan a route here anyway, so the announce window is the whole
        answer.
        """
        bot = self._bot
        if bot.graph is None or bot.planner is None:
            return 2 * int(config.T_ANNOUNCE * 1000)
        return claim_window_ms(bot.graph.node_spacing, bot.planner.kinematics)

    def _rank(self) -> int:
        bot = self._bot
        return prio.yield_priority(
            has_job=bot.current_job is not None,
            carrying=bot.cargo_state in CS_CARRYING,
        )

    # ---- Telling the neighbours ---------------------------------------------

    def _announce(self, now: int) -> None:
        bot = self._bot
        held = [claim.node_id for claim in bot.ledger.mine]
        if not held:
            return

        # A follower's roster includes its own record — the leader sends the whole
        # region back, us included — so filter ourselves out or we announce to
        # ourselves every tick. The ledger would ignore it, but the RPC is real.
        peers = {
            p.bot_id: p.latest_node_id
            for p in bot.peer_table.all_peers()
            if p.latest_node_id and p.bot_id != bot.bot_id
        }
        nearby = in_claim_range(
            bot.map,
            claimed_node_ids=held,
            peers=peers,
            reach_hops=config.RESERVATION_REACH_HOPS,
        )
        if not nearby:
            return

        message = to_proto(bot.ledger.announcement(now))
        metadata = bot.rpc_metadata()
        addresses = {p.bot_id: p.address for p in bot.peer_table.all_peers()}
        for bot_id in nearby:
            address = addresses.get(bot_id)
            if not address:
                continue
            try:
                pool.stub(address, fleet_pb2_grpc.ReservationServiceStub).Announce(
                    message, timeout=config.T_ANNOUNCE, metadata=metadata
                )
            except grpc.RpcError as error:
                log.debug(
                    "bot-%d: could not announce to bot-%d: %s",
                    bot.bot_id, bot_id, error.code(),
                )

        # A neighbour that has drifted out of claim range stops mattering to us.
        for bot_id in set(bot.ledger.neighbours) - set(nearby):
            bot.ledger.forget(bot_id)
