"""Reservation table: clock correction, widening, merging and safe intervals."""

from __future__ import annotations

import pytest
from tests.planning_maps import line, make_graph

from planning.intervals import ReservationTable, _complement, _merge
from planning.types import INF_MS, Config, PeerView, Reservation

CFG = Config(safety_ms=0, skew_bound_ms=100, follow_gap_ms=0)


def peer(bot_id: int, *claims, **kwargs) -> PeerView:
    return PeerView(
        bot_id=bot_id,
        reservations=tuple(
            Reservation(node_id=n, t_in=a, t_out=b) for n, a, b in claims
        ),
        **kwargs,
    )


@pytest.fixture
def graph():
    return make_graph(line(6))


def test_an_unclaimed_node_is_free_forever(graph):
    table = ReservationTable(graph, [], now=0, config=CFG)
    assert table.safe_intervals(3) == ((0, INF_MS),)
    assert table.blocked(3) == ()
    assert table.is_free(3, 0, 10**9)


def test_a_claim_carves_a_hole_in_the_safe_intervals(graph):
    table = ReservationTable(graph, [peer(1, (2, 1000, 2000))], now=0, config=CFG)
    node = graph.index(2)
    assert table.blocked(node) == ((1000, 2000),)
    assert table.safe_intervals(node) == ((0, 1000), (2000, INF_MS))
    assert not table.is_free(node, 1500, 1600)
    assert table.is_free(node, 0, 1000)
    assert table.is_free(node, 2000, 5000)


def test_overlapping_claims_from_different_peers_merge(graph):
    table = ReservationTable(
        graph, [peer(1, (2, 1000, 2000)), peer(2, (2, 1500, 3000))], now=0, config=CFG
    )
    node = graph.index(2)
    assert table.blocked(node) == ((1000, 3000),)
    assert table.blockers(node, 1200, 1300) == (1,)
    assert sorted(table.blockers(node, 1600, 1700)) == [1, 2]


def test_safety_margin_widens_every_claim(graph):
    table = ReservationTable(
        graph, [peer(1, (2, 1000, 2000))], now=0, config=Config(safety_ms=250)
    )
    assert table.blocked(graph.index(2)) == ((750, 2250),)


def test_a_desynced_peer_is_given_extra_room(graph):
    config = Config(safety_ms=0, skew_bound_ms=400)
    trusted = ReservationTable(graph, [peer(1, (2, 1000, 2000))], now=0, config=config)
    suspect = ReservationTable(
        graph, [peer(1, (2, 1000, 2000), desynced=True)], now=0, config=config
    )
    assert trusted.blocked(graph.index(2)) == ((1000, 2000),)
    assert suspect.blocked(graph.index(2)) == ((600, 2400),)


def test_peer_timestamps_are_moved_onto_the_local_clock(graph):
    table = ReservationTable(
        graph, [peer(1, (2, 1000, 2000), clock_offset_ms=5000)], now=0, config=CFG
    )
    assert table.blocked(graph.index(2)) == ((6000, 7000),)


def test_the_robot_excludes_its_own_claims(graph):
    peers = [peer(1, (2, 1000, 2000)), peer(42, (3, 1000, 2000))]
    table = ReservationTable(graph, peers, now=0, config=CFG, exclude_bot_id=42)
    assert table.blocked(graph.index(2)) == ((1000, 2000),)
    assert table.blocked(graph.index(3)) == ()


def test_claims_that_already_expired_do_not_shorten_the_future(graph):
    table = ReservationTable(graph, [peer(1, (2, 0, 500))], now=1000, config=CFG)
    assert table.safe_intervals(graph.index(2)) == ((1000, INF_MS),)


def test_a_claim_covering_now_leaves_no_interval_at_now(graph):
    table = ReservationTable(graph, [peer(1, (2, 0, 5000))], now=1000, config=CFG)
    node = graph.index(2)
    assert table.safe_intervals(node) == ((5000, INF_MS),)
    assert table.interval_containing(node, 1000) is None
    assert table.interval_containing(node, 6000) == (5000, INF_MS)


def test_claims_on_nodes_this_map_does_not_have_are_counted_not_swallowed(graph):
    # A peer running a different map revision. There is no node here to block, but
    # silently dropping the claim would hide a real version skew.
    table = ReservationTable(graph, [peer(1, (999, 0, 1000))], now=0, config=CFG)
    assert table.unknown_node_claims == 1
    assert table.blocked_nodes == frozenset()


def test_a_handoff_at_the_exact_release_instant_is_allowed(graph):
    # The safe interval starts where the blocked one ends, so `is_free` must agree.
    # The physical clearance comes from `safety_ms`, applied on both sides already.
    table = ReservationTable(graph, [peer(1, (2, 1000, 2000))], now=0, config=CFG)
    node = graph.index(2)
    assert table.is_free(node, 2000, 3000)
    assert table.is_free(node, 0, 1000)
    assert not table.is_free(node, 1999, 3000)


def test_reservation_table_reports_which_nodes_are_blocked(graph):
    table = ReservationTable(graph, [peer(1, (2, 0, 10), (4, 0, 10))], now=0, config=CFG)
    assert table.blocked_nodes == {graph.index(2), graph.index(4)}
    assert "2 blocked nodes" in repr(table)


# -- interval helpers --------------------------------------------------------


def test_merge_coalesces_overlapping_and_touching_intervals():
    assert _merge([(10, 20), (15, 25), (30, 40)]) == ((10, 25), (30, 40))
    assert _merge([(0, 10), (10, 20)]) == ((0, 20),)
    assert _merge([]) == ()
    assert _merge([(5, 6), (0, 1)]) == ((0, 1), (5, 6))


def test_merge_absorbs_a_fully_contained_interval():
    assert _merge([(0, 100), (10, 20)]) == ((0, 100),)


def test_complement_runs_from_now_to_forever():
    assert _complement(((10, 20), (30, 40)), 0) == ((0, 10), (20, 30), (40, INF_MS))
    assert _complement((), 5) == ((5, INF_MS),)
    assert _complement(((0, 50),), 10) == ((50, INF_MS),)
