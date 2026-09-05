"""Reservations: the rules, and the wire they travel on (PROTOCOL.md §15).

The property under test throughout is that two bots reach the same conclusion
from their own state alone. Nothing here may need a shared table, a shared clock,
or a round trip — those are exactly what §7 rules out by requiring collision
avoidance to survive a leader going away.

These are the fast tier. The proof that it works between separate processes on a
real network lives in `test_docker.py`.
"""
from __future__ import annotations

import grpc
import pytest

import config
from peers.table import Peer
from proto import fleet_pb2, fleet_pb2_grpc
from reservations.claims import Announce, Claim, Window, contests
from reservations.ledger import ReservationLedger
from reservations.server import to_proto
from reservations.vicinity import in_claim_range
from tests.conftest import md, wait_until
from warehouse.map import NullMap, WarehouseMap

PERIOD, TTL = 1000, 3000


def ledger(bot_id: int) -> ReservationLedger:
    return ReservationLedger(bot_id, announce_period_ms=PERIOD, ttl_ms=TTL)


def claim(bot_id: int, node_id: int = 10, start: int = 0, end: int = 1000, rank: int = 0) -> Claim:
    return Claim(bot_id=bot_id, node_id=node_id, start_ms=start, end_ms=end, rank=rank)


# =============================================================================
# Who gives way

def test_lower_bot_id_wins_an_otherwise_equal_clash():
    assert claim(1).outranks(claim(3))
    assert not claim(3).outranks(claim(1))


def test_carrying_cargo_beats_being_free_whatever_the_ids():
    assert claim(9, rank=2).outranks(claim(1, rank=0))


def test_both_sides_of_a_clash_reach_the_same_verdict():
    everyone = [claim(b, rank=r) for b in (1, 2, 7) for r in (0, 1, 2)]
    for a in everyone:
        for b in everyone:
            if a.order != b.order:
                assert a.outranks(b) != b.outranks(a)


def test_a_node_may_be_taken_the_instant_it_is_released():
    held = claim(1, start=0, end=1000)
    assert held.overlaps(10, 500, 1500)
    assert not held.overlaps(10, 1000, 2000)
    assert not held.overlaps(11, 0, 1000)


def test_a_bot_does_not_clash_with_itself():
    assert contests((claim(1),), (claim(1),)) == ()


# =============================================================================
# Clocks that disagree

def test_windows_travel_as_offsets_and_are_stamped_on_arrival():
    stamped = Announce(bot_id=7, seq=1, rank=1, windows=(Window(10, 200, 2400),)).stamp(5_000, 5_200)
    assert (stamped[0].start_ms, stamped[0].end_ms) == (5_200, 7_400)
    assert stamped[0].rank == 1


def test_the_same_announcement_means_the_same_thing_on_any_clock():
    announce = Announce(bot_id=7, seq=1, windows=(Window(10, 0, 1000),))
    here, far_future = announce.stamp(0, 0), announce.stamp(10**9, 10**9)
    assert here[0].end_ms - here[0].start_ms == far_future[0].end_ms - far_future[0].start_ms


# =============================================================================
# The ledger

def test_a_fresh_claim_does_not_count_until_a_round_has_passed():
    # The whole safety story: two bots claiming the same node at the same instant
    # have not heard each other, and acting at once would collide.
    led = ledger(1)
    led.propose([(10, 0, 5000)], now=0)
    assert not led.may_enter(10, 0, 5000, now=0)
    assert not led.may_enter(10, 0, 5000, now=PERIOD - 1)
    assert led.may_enter(10, 0, 5000, now=PERIOD)


def test_carrying_on_holding_a_node_does_not_restart_the_wait():
    # Re-claiming every tick must not keep the claim provisional forever.
    led = ledger(1)
    led.propose([(10, 0, 2000)], now=0)
    first = led.mine[0].effective_at_ms
    led.propose([(10, 500, 2500)], now=500)
    led.propose([(10, 1000, 3000)], now=1000)
    assert led.mine[0].effective_at_ms == first
    assert led.may_enter(10, 1000, 3000, now=1000)


def test_a_new_claim_on_the_same_node_after_a_gap_waits_its_turn():
    led = ledger(1)
    led.propose([(10, 0, 2000)], now=0)
    led.propose([(10, 90_000, 92_000)], now=1000)
    assert led.mine[0].effective_at_ms == 1000 + PERIOD


def test_a_claim_is_refused_when_a_better_ranked_neighbour_holds_it():
    led = ledger(3)
    led.receive(Announce(bot_id=1, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert not led.propose([(10, 500, 1500)], now=0)
    assert led.mine == ()


def test_a_claim_is_granted_over_a_neighbour_we_outrank():
    led = ledger(1)
    led.receive(Announce(bot_id=3, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert led.propose([(10, 500, 1500)], now=0)


def test_claims_are_all_or_nothing():
    led = ledger(3)
    led.receive(Announce(bot_id=1, seq=1, windows=(Window(11, 0, 1000),)), now=0)
    assert not led.propose([(10, 0, 1000), (11, 0, 1000)], now=0)
    assert led.mine == (), "a half-granted route is one the robot cannot finish"


def test_the_winner_waits_for_the_retraction_not_just_for_the_verdict():
    # Deliberately stricter than the ordering requires: the loser may not have
    # heard yet, and "I should win" is not "the node is free".
    winner = ledger(1)
    winner.propose([(10, 0, 5000)], now=0)
    winner.receive(Announce(bot_id=3, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert not winner.may_enter(10, 0, 5000, now=PERIOD)
    winner.receive(Announce(bot_id=3, seq=2, windows=()), now=PERIOD)
    assert winner.may_enter(10, 0, 5000, now=PERIOD)


def test_two_bots_claiming_at_once_settle_with_no_round_trip():
    a, b = ledger(1), ledger(3)
    assert a.propose([(10, 0, 2000)], now=0)
    assert b.propose([(10, 1000, 3000)], now=0), "neither has heard the other yet"

    a.receive(b.announcement(0), now=0)
    b.receive(a.announcement(0), now=0)

    assert not a.lost()
    assert b.lost(), "bot 3 gives way to bot 1"
    assert not a.may_enter(10, 0, 2000, now=0), "and neither has moved"


def test_an_empty_announcement_frees_the_nodes_at_once():
    led = ledger(1)
    led.receive(Announce(bot_id=2, seq=1, windows=(Window(10, 0, 5000),)), now=0)
    assert led.blockers(10, 0, 1000) == (2,)
    led.receive(Announce(bot_id=2, seq=2, windows=()), now=0)
    assert led.blockers(10, 0, 1000) == ()


def test_a_neighbour_that_goes_quiet_stops_blocking():
    led = ledger(1)
    led.receive(Announce(bot_id=2, seq=1, ttl_ms=TTL, windows=(Window(10, 0, 90_000),)), now=0)
    assert led.expire(now=TTL - 1) == ()
    assert led.expire(now=TTL) == (2,)
    assert led.blockers(10, 0, 1000) == ()


def test_a_ledger_ignores_its_own_announcement():
    led = ledger(1)
    led.propose([(10, 0, 1000)], now=0)
    led.receive(led.announcement(0), now=0)
    assert led.neighbours == ()


def test_withdrawing_releases_everything_and_bumps_the_sequence():
    led = ledger(1)
    led.propose([(10, 0, 1000)], now=0)
    before = led.announcement(0).seq
    led.withdraw()
    assert led.mine == ()
    assert led.announcement(0).seq > before


# =============================================================================
# Who we tell

@pytest.fixture(scope="module")
def warehouse():
    return WarehouseMap.load(config.WAREHOUSE_MAP)


def test_only_bots_near_a_node_we_hold_hear_from_us(warehouse):
    near = in_claim_range(warehouse, claimed_node_ids=[100], peers={4: 100, 6: 700}, reach_hops=8)
    assert near == (4,), "bot 6 is far away and cannot contest anything we hold"


def test_holding_nothing_means_telling_nobody(warehouse):
    assert in_claim_range(warehouse, claimed_node_ids=[], peers={4: 100}, reach_hops=8) == ()


def test_a_node_this_map_does_not_have_is_not_a_neighbour(warehouse):
    assert in_claim_range(warehouse, claimed_node_ids=[100], peers={9: 10**9}, reach_hops=8) == ()


def test_without_a_map_we_tell_everyone_rather_than_nobody():
    # NullMap reports every distance as 0. Chattier, still correct — the same way
    # job dispatch degrades when it cannot rank by geography.
    assert in_claim_range(NullMap(), claimed_node_ids=[1], peers={4: 2, 6: 3}, reach_hops=8) == (4, 6)


# =============================================================================
# Over real gRPC

def announce_to(target, sender_bot_id: int, region_id: int, windows, rank: int = 0):
    stub = fleet_pb2_grpc.ReservationServiceStub(
        grpc.insecure_channel(target.address)
    )
    return stub.Announce(
        to_proto(Announce(bot_id=sender_bot_id, seq=1, rank=rank, ttl_ms=TTL, windows=windows)),
        timeout=5.0,
        metadata=md(sender_bot_id, region_id),
    )


def test_an_announcement_crosses_the_wire_into_a_peers_ledger(fleet):
    receiver = fleet.bot(1, region_id=14)
    announce_to(receiver, sender_bot_id=2, region_id=14, windows=(Window(412, 0, 2000),))
    assert wait_until(lambda: receiver.ledger.neighbours == (2,))
    # The receiver stamped its own clock on arrival, so assert the shape of the
    # window rather than absolute instants the sender never named.
    (arrived,) = receiver.ledger.peer_claims()
    assert arrived.node_id == 412
    assert arrived.end_ms - arrived.start_ms == 2000
    assert receiver.ledger.blockers(412, arrived.start_ms, arrived.end_ms) == (2,)


def test_a_bot_from_another_region_is_refused(fleet):
    receiver = fleet.bot(1, region_id=14)
    with pytest.raises(grpc.RpcError) as caught:
        announce_to(receiver, sender_bot_id=2, region_id=99, windows=(Window(412, 0, 2000),))
    assert caught.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert receiver.ledger.neighbours == ()


def test_an_announcement_without_identity_is_refused(fleet):
    receiver = fleet.bot(1, region_id=14)
    stub = fleet_pb2_grpc.ReservationServiceStub(grpc.insecure_channel(receiver.address))
    with pytest.raises(grpc.RpcError) as caught:
        stub.Announce(fleet_pb2.ReservationAnnounce(bot_id=2, seq=1), timeout=5.0)
    assert caught.value.code() == grpc.StatusCode.UNAUTHENTICATED


def test_the_run_loop_step_claims_where_the_robot_is_and_tells_the_neighbours(fleet):
    """The whole tick, end to end: claim underfoot, then announce it."""
    mover, watcher = fleet.bot(1, region_id=14), fleet.bot(2, region_id=14)
    mover.latest_node_id, watcher.latest_node_id = 412, 413
    mover.peer_table.upsert(
        Peer(bot_id=2, address=watcher.address, priority=watcher.priority, latest_node_id=413)
    )

    mover._reservations.tick()

    assert mover.ledger.mine[0].node_id == 412, "claimed the node it is standing on"
    assert wait_until(lambda: watcher.ledger.neighbours == (1,)), "and told the neighbour"
    assert [c.node_id for c in watcher.ledger.peer_claims()] == [412]


def test_a_bot_that_has_not_scanned_a_qr_code_claims_nothing(fleet):
    bot = fleet.bot(1, region_id=14)
    bot.latest_node_id = 0
    bot._reservations.tick()
    assert bot.ledger.mine == (), "it cannot say where it is, so it says nothing"


def test_a_far_away_bot_is_not_told(fleet):
    mover, distant = fleet.bot(1, region_id=14), fleet.bot(2, region_id=14)
    mover.latest_node_id = 412
    mover.peer_table.upsert(
        Peer(bot_id=2, address=distant.address, priority=distant.priority, latest_node_id=880)
    )
    mover._reservations.tick()
    assert distant.ledger.neighbours == ()


def test_an_announcement_that_arrives_late_is_ignored():
    # Each announcement carries the sender's whole claim set, so applying a stale
    # one would resurrect claims it has since given up.
    led = ledger(1)
    led.receive(Announce(bot_id=2, seq=5, windows=(Window(10, 0, 1000),)), now=0)
    led.receive(Announce(bot_id=2, seq=4, windows=(Window(11, 0, 1000),)), now=0)
    assert [c.node_id for c in led.peer_claims()] == [10]
    led.receive(Announce(bot_id=2, seq=6, windows=(Window(11, 0, 1000),)), now=0)
    assert [c.node_id for c in led.peer_claims()] == [11]


def test_forgetting_a_neighbour_forgets_its_sequence_too():
    # Otherwise a bot that leaves and comes back renumbered would be ignored.
    led = ledger(1)
    led.receive(Announce(bot_id=2, seq=9, windows=(Window(10, 0, 1000),)), now=0)
    led.forget(2)
    led.receive(Announce(bot_id=2, seq=1, windows=(Window(11, 0, 1000),)), now=0)
    assert [c.node_id for c in led.peer_claims()] == [11]


def test_a_bot_does_not_announce_to_itself(fleet):
    # A follower's roster comes back from the leader with its own record in it.
    bot = fleet.bot(1, region_id=14)
    bot.latest_node_id = 412
    bot.peer_table.upsert(
        Peer(bot_id=1, address=bot.address, priority=bot.priority, latest_node_id=412)
    )
    bot._reservations.tick()
    assert bot.ledger.neighbours == (), "it would be ignored, but the RPC is still real"
