"""The peer plane: claims, arbitration, and the per-bot ledger.

The property under test throughout is that two bots reach the same conclusion from
their own state alone. Nothing here may depend on a shared table, a shared clock, or
a round trip.
"""

from __future__ import annotations

import pytest
from conftest import line, make_graph

from spore_planner.p2p import (
    Announcement,
    Claim,
    Ledger,
    Window,
    contests,
    in_claim_range,
    neighbours,
    radius_for,
)


def claim(bot_id, node_id=10, start=0, end=1000, rank=0, effective=0) -> Claim:
    return Claim(
        bot_id=bot_id, node_id=node_id, start_ms=start, end_ms=end,
        rank=rank, effective_at_ms=effective,
    )


# -- the ordering ------------------------------------------------------------


def test_a_lower_bot_id_wins_an_otherwise_equal_contest():
    assert claim(1).outranks(claim(3))
    assert not claim(3).outranks(claim(1))


def test_right_of_way_beats_bot_id():
    # A robot carrying cargo is not asked to give way to an idle one, whatever
    # their ids. This is the leader's `yield_priority`, used as-is.
    carrying = claim(9, rank=2)
    idle = claim(1, rank=0)
    assert carrying.outranks(idle)


def test_the_ordering_is_total_and_agreed_by_both_sides():
    everyone = [claim(b, rank=r) for b in (1, 2, 7) for r in (0, 1, 2)]
    for a in everyone:
        for b in everyone:
            if a.order != b.order:
                assert a.outranks(b) != b.outranks(a), "both must reach the same verdict"


def test_overlap_needs_to_be_strictly_positive():
    # Same convention as the planner's safe intervals: one robot may take a node
    # the instant another releases it.
    held = claim(1, start=0, end=1000)
    assert held.overlaps(10, 500, 1500)
    assert not held.overlaps(10, 1000, 2000)
    assert not held.overlaps(11, 0, 1000), "a different node is not a contest"


def test_contests_are_found_in_a_stable_order():
    mine = (claim(1, node_id=10), claim(1, node_id=11))
    theirs = (claim(2, node_id=11), claim(3, node_id=10))
    found = contests(mine, theirs)
    assert [c.node_id for c in found] == [10, 11]
    assert contests(mine, theirs) == found


def test_a_bot_does_not_contest_itself():
    assert contests((claim(1),), (claim(1),)) == ()


# -- relative windows --------------------------------------------------------


def test_windows_travel_as_offsets_and_are_stamped_on_arrival():
    # No shared clock anywhere: the sender says "+200 to +2400", the receiver
    # decides what that means in its own time.
    announcement = Announcement(bot_id=7, seq=1, rank=1, windows=(Window(10, 200, 2400),))
    claims = announcement.stamp(received_at_ms=5_000, effective_at_ms=5_200)
    assert claims[0].start_ms == 5_200
    assert claims[0].end_ms == 7_400
    assert claims[0].rank == 1


def test_the_same_announcement_lands_correctly_on_clocks_that_disagree():
    announcement = Announcement(bot_id=7, seq=1, windows=(Window(10, 0, 1000),))
    early = announcement.stamp(received_at_ms=0, effective_at_ms=0)
    late = announcement.stamp(received_at_ms=1_000_000, effective_at_ms=1_000_000)
    assert early[0].end_ms - early[0].start_ms == late[0].end_ms - late[0].start_ms


# -- the ledger --------------------------------------------------------------


def test_an_uncontested_claim_is_granted():
    ledger = Ledger(1, announce_period_ms=200)
    assert ledger.propose([(10, 0, 1000)], now=0)
    assert len(ledger.mine) == 1


def test_a_claim_is_provisional_until_one_announce_period_has_passed():
    # The heart of it. Acting the instant a claim is made would mean driving into
    # a node whose rival claim is still in flight.
    ledger = Ledger(1, announce_period_ms=200)
    ledger.propose([(10, 0, 1000)], now=0)
    assert not ledger.may_enter(10, 0, 1000, now=0)
    assert not ledger.may_enter(10, 0, 1000, now=199)
    assert ledger.may_enter(10, 0, 1000, now=200)


def test_a_claim_is_refused_when_a_better_ranked_neighbour_holds_it():
    ledger = Ledger(3, announce_period_ms=200)
    ledger.receive(Announcement(bot_id=1, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert not ledger.propose([(10, 500, 1500)], now=0)
    assert ledger.mine == ()


def test_a_claim_is_granted_over_a_neighbour_we_outrank():
    ledger = Ledger(1, announce_period_ms=200)
    ledger.receive(Announcement(bot_id=3, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert ledger.propose([(10, 500, 1500)], now=0)


def test_claims_are_all_or_nothing():
    ledger = Ledger(3, announce_period_ms=200)
    ledger.receive(Announcement(bot_id=1, seq=1, windows=(Window(11, 0, 1000),)), now=0)
    assert not ledger.propose([(10, 0, 1000), (11, 0, 1000)], now=0)
    assert ledger.mine == (), "a half-granted route is one the robot cannot finish"


def test_a_bot_waits_for_the_withdrawal_rather_than_assuming_it_won():
    # Deliberately stricter than the ordering requires. The loser may not have
    # heard yet, and "I should win" is not the same as "the node is free".
    winner = Ledger(1, announce_period_ms=200)
    winner.propose([(10, 0, 1000)], now=0)
    winner.receive(Announcement(bot_id=3, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    assert not winner.may_enter(10, 0, 1000, now=200)
    # Once the loser's retraction arrives, the way is clear.
    winner.receive(Announcement(bot_id=3, seq=2, windows=()), now=200)
    assert winner.may_enter(10, 0, 1000, now=200)


def test_two_bots_claiming_at_once_settle_without_a_round_trip():
    a, b = Ledger(1, announce_period_ms=200), Ledger(3, announce_period_ms=200)
    assert a.propose([(10, 0, 1000)], now=0)
    assert b.propose([(10, 500, 1500)], now=0), "neither has heard the other yet"

    a.receive(b.announcement(0), now=0)
    b.receive(a.announcement(0), now=0)

    assert not a.lost(), "bot 1 outranks bot 3"
    assert b.lost(), "bot 3 must give way"
    # And crucially neither has moved: both claims were still provisional.
    assert not a.may_enter(10, 0, 1000, now=0)


def test_an_empty_announcement_frees_a_neighbours_nodes_at_once():
    ledger = Ledger(1)
    ledger.receive(Announcement(bot_id=2, seq=1, windows=(Window(10, 0, 5000),)), now=0)
    assert ledger.blockers(10, 0, 1000) == (2,)
    ledger.receive(Announcement(bot_id=2, seq=2, windows=()), now=0)
    assert ledger.blockers(10, 0, 1000) == ()


def test_a_neighbour_that_goes_quiet_stops_blocking():
    ledger = Ledger(1, ttl_ms=600)
    ledger.receive(Announcement(bot_id=2, seq=1, ttl_ms=600, windows=(Window(10, 0, 9000),)), now=0)
    assert ledger.expire(now=500) == ()
    assert ledger.blockers(10, 0, 1000) == (2,)
    assert ledger.expire(now=600) == (2,)
    assert ledger.blockers(10, 0, 1000) == ()


def test_withdrawing_releases_everything_and_bumps_the_sequence():
    ledger = Ledger(1)
    ledger.propose([(10, 0, 1000)], now=0)
    before = ledger.announcement(0).seq
    ledger.withdraw()
    assert ledger.mine == ()
    assert ledger.announcement(0).seq > before
    assert ledger.announcement(0).windows == ()


def test_a_ledger_ignores_its_own_announcement():
    ledger = Ledger(1)
    ledger.propose([(10, 0, 1000)], now=0)
    ledger.receive(ledger.announcement(0), now=0)
    assert ledger.neighbours == ()


def test_forgetting_a_neighbour_drops_its_claims():
    ledger = Ledger(1)
    ledger.receive(Announcement(bot_id=2, seq=1, windows=(Window(10, 0, 1000),)), now=0)
    ledger.forget(2)
    assert ledger.neighbours == ()
    assert ledger.peer_claims() == ()


def test_neighbour_claims_convert_to_planner_reservations():
    ledger = Ledger(1)
    ledger.receive(Announcement(bot_id=2, seq=1, windows=(Window(10, 200, 2400),)), now=1000)
    by_bot = ledger.reservations_by_bot()
    assert list(by_bot) == [2]
    reservation = by_bot[2][0]
    assert (reservation.node_id, reservation.t_in, reservation.t_out) == (10, 1200, 3400)


def test_announcement_offsets_are_relative_to_the_moment_of_sending():
    ledger = Ledger(1)
    ledger.propose([(10, 5_000, 7_000)], now=4_000)
    windows = ledger.announcement(4_000).windows
    assert (windows[0].start_offset_ms, windows[0].end_offset_ms) == (1_000, 3_000)


# -- vicinity ----------------------------------------------------------------


def test_the_radius_is_twice_the_commit_horizon():
    # Claims reach k_commit hops from each bot, so they can only meet within 2x.
    assert radius_for(8) == 16


def test_neighbours_are_those_within_the_radius_nearest_first():
    graph = make_graph(line(20))
    found = neighbours(
        graph, at_node_id=graph.id_of(10),
        peers={7: graph.id_of(12), 8: graph.id_of(19), 9: graph.id_of(11)},
        radius=3,
    )
    assert found == (9, 7), "bot 8 is nine hops away and cannot matter"


def test_vicinity_is_symmetric():
    graph = make_graph(line(20))
    a, b = graph.id_of(4), graph.id_of(9)
    assert neighbours(graph, at_node_id=a, peers={2: b}, radius=6) == (2,)
    assert neighbours(graph, at_node_id=b, peers={1: a}, radius=6) == (1,)


def test_claim_range_is_measured_from_the_nodes_actually_held():
    # Tighter than a ball around the bot, and still exact: a peer can only reach
    # our claims if it is within its own reach of one of them.
    graph = make_graph(line(30))
    peers = {5: graph.id_of(20)}
    near = in_claim_range(
        graph, claimed_node_ids=[graph.id_of(i) for i in range(14, 18)],
        peers=peers, reach_hops=4,
    )
    far = in_claim_range(
        graph, claimed_node_ids=[graph.id_of(i) for i in range(0, 4)],
        peers=peers, reach_hops=4,
    )
    assert near == (5,)
    assert far == ()


def test_claim_range_with_nothing_held_tells_nobody():
    graph = make_graph(line(10))
    assert in_claim_range(graph, claimed_node_ids=[], peers={1: graph.id_of(0)}, reach_hops=5) == ()


def test_vicinity_tolerates_nodes_this_map_does_not_have():
    graph = make_graph(line(5))
    assert neighbours(graph, at_node_id=999, peers={1: graph.id_of(0)}, radius=3) == ()
    assert neighbours(graph, at_node_id=graph.id_of(0), peers={1: 999}, radius=3) == ()


@pytest.mark.parametrize("rank", [0, 1, 2])
def test_rank_travels_with_the_announcement(rank):
    ledger = Ledger(1)
    ledger.propose([(10, 0, 1000)], now=0, rank=rank)
    assert ledger.announcement(0).rank == rank
