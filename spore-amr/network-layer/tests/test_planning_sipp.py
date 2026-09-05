"""The safe-interval search.

Two families of check: that the timing it emits is physically coherent and honours
the overlapping-claim invariant, and that it actually reacts to traffic -- waiting
when there is no way round, going round when there is, and letting the energy state
decide which.
"""

from __future__ import annotations

import pytest
from tests.planning_maps import line, make_graph

from planning.cost import CostModel, EnergyState
from planning.intervals import ReservationTable
from planning.sipp import search
from planning.types import Config, PeerView, PlanStatus, Reservation
from planning.geometry import Heading

CFG = Config()
SPACING = 200


def model(state: EnergyState = EnergyState.OK, **kwargs) -> CostModel:
    return CostModel.for_state(SPACING, state, **kwargs)


def table(graph, peers=(), *, now=0, config=CFG) -> ReservationTable:
    return ReservationTable(graph, peers, now=now, config=config)


def blocking(bot_id: int, node_id: int, t_in: int, t_out: int) -> PeerView:
    return PeerView(
        bot_id=bot_id, reservations=(Reservation(node_id=node_id, t_in=t_in, t_out=t_out),)
    )


def ladder():
    """Two parallel five-node lanes, joined at every rung."""
    cells = {(x, y): "PT" for x in range(5) for y in range(2)}
    graph = make_graph(cells)
    ids = {cell: i for i, cell in enumerate(sorted(cells))}
    return graph, ids, {v: k for k, v in ids.items()}


def run(graph, tbl, start, goals, *, cost=None, heading=Heading.E, moving=False, **kwargs):
    return search(
        graph,
        tbl,
        start=start,
        goals=goals,
        cost_model=cost or model(),
        config=kwargs.pop("config", CFG),
        start_heading=heading,
        moving=moving,
        **kwargs,
    )


# -- basic outcomes ----------------------------------------------------------


def test_a_clear_corridor_is_traversed_straight_through():
    graph = make_graph(line(6))
    result = run(graph, table(graph), 0, [5])
    assert result.status is PlanStatus.OK
    assert result.nodes == (0, 1, 2, 3, 4, 5)
    assert result.waits == (0, 0, 0, 0, 0)


def test_starting_on_the_goal_reports_already_there():
    graph = make_graph(line(4))
    result = run(graph, table(graph), 2, [2])
    assert result.status is PlanStatus.ALREADY_THERE
    assert result.nodes == (2,)


def test_a_peer_standing_on_our_own_node_is_reported_not_routed_around():
    graph = make_graph(line(4))
    tbl = table(graph, [blocking(1, 0, 0, 10_000)])
    assert run(graph, tbl, 0, [3]).status is PlanStatus.START_BLOCKED


def test_an_unreachable_goal_is_reported():
    graph = make_graph({(0, 0): "PT", (1, 0): "PT", (3, 0): "PT", (4, 0): "PT"})
    assert run(graph, table(graph), 0, [graph.index(3)]).status is PlanStatus.UNREACHABLE


def test_an_empty_goal_set_is_reported():
    graph = make_graph(line(3))
    assert run(graph, table(graph), 0, []).status is PlanStatus.NO_GOAL_AVAILABLE


def test_the_search_budget_is_enforced():
    graph = make_graph({(x, y): "PT" for x in range(12) for y in range(12)})
    result = run(graph, table(graph), 0, [graph.n - 1], config=Config(max_expansions=3))
    assert result.status is PlanStatus.SEARCH_EXHAUSTED


def test_multiple_goals_resolve_to_the_nearest():
    graph = make_graph(line(9))
    result = run(graph, table(graph), 4, [0, 6])
    assert result.nodes[-1] == 6


# -- timing coherence --------------------------------------------------------


def test_timing_follows_the_kinematic_model():
    graph = make_graph(line(4))
    kin = model().kinematics
    result = run(graph, table(graph), 0, [3])
    # First hop accelerates from rest; the rest are straight passes at cruise.
    assert result.arrivals[1] - result.arrivals[0] == kin.half_stop_ms() + kin.cruise_ms(SPACING)
    assert result.arrivals[2] - result.arrivals[1] == kin.cruise_ms(SPACING)
    assert result.duration_ms == result.arrivals[-1] + kin.half_stop_ms()


def test_departures_precede_arrivals_by_exactly_one_traversal():
    graph = make_graph(line(5))
    kin = model().kinematics
    result = run(graph, table(graph), 0, [4])
    for i, departure in enumerate(result.departures):
        assert result.arrivals[i + 1] - departure == kin.cruise_ms(SPACING)
        assert departure >= result.arrivals[i]


def test_the_overlapping_claim_invariant_holds_across_every_hop():
    # A robot holds both endpoints for the whole traversal, so the window it holds
    # a node for must reach at least until it has entered the next one. This is what
    # makes node-only reservations enough to catch head-on swaps.
    graph = make_graph(line(6))
    tbl = table(graph, [blocking(1, 3, 0, 9_000)])
    result = run(graph, tbl, 0, [5])
    for i in range(len(result.nodes) - 1):
        t_out_of_this_node = result.arrivals[i + 1]
        t_in_of_next_node = result.departures[i]
        assert t_out_of_this_node >= t_in_of_next_node


def test_an_unknown_heading_is_charged_as_a_full_reversal():
    # Conservative on purpose: guessing low here would under-size a reservation.
    graph = make_graph(line(4))
    known = run(graph, table(graph), 0, [3], heading=Heading.E)
    unknown = run(graph, table(graph), 0, [3], heading=None)
    assert unknown.arrivals[1] > known.arrivals[1]
    assert unknown.arrivals[1] - known.arrivals[1] == model().kinematics.turn_ms(2)


def test_a_straight_route_is_preferred_over_an_equal_length_staircase():
    # Same hop count either way, but turns cost time and charge.
    cells = {(x, 0): "PT" for x in range(4)} | {(x, 1): "PT" for x in range(4)}
    graph = make_graph(cells)
    ids = {cell: i for i, cell in enumerate(sorted(cells))}
    result = run(graph, table(graph), ids[(0, 0)], [ids[(3, 0)]])
    assert [ids[(x, 0)] for x in range(4)] == list(result.nodes)


# -- reacting to traffic -----------------------------------------------------


def test_with_no_way_round_the_robot_waits_for_the_node_to_clear():
    graph = make_graph(line(6))
    tbl = table(graph, [blocking(1, 3, 0, 20_000)])
    result = run(graph, tbl, 0, [5])
    assert result.nodes == (0, 1, 2, 3, 4, 5)
    assert sum(result.waits) > 0
    # It enters the contested node only once the claim has lapsed.
    entered_at = result.departures[result.nodes.index(3) - 1]
    assert entered_at >= tbl.safe_intervals(3)[0][0]


def test_the_robot_goes_round_a_long_block_when_a_detour_exists():
    graph, ids, cell_of = ladder()
    tbl = table(graph, [blocking(9, ids[(2, 0)], 0, 60_000)])
    result = run(graph, tbl, ids[(0, 0)], [ids[(4, 0)]])
    assert ids[(2, 0)] not in result.nodes
    assert any(cell_of[n][1] == 1 for n in result.nodes)


def test_the_planned_path_never_overlaps_a_peer_claim():
    graph, ids, _ = ladder()
    tbl = table(graph, [blocking(9, ids[(2, 0)], 0, 30_000)])
    result = run(graph, tbl, ids[(0, 0)], [ids[(4, 0)]])
    for i, node in enumerate(result.nodes):
        t_in = tbl.now if i == 0 else result.departures[i - 1]
        t_out = result.arrivals[i + 1] if i + 1 < len(result.nodes) else result.arrivals[i]
        assert tbl.is_free(node, t_in, t_out), f"node {node} overlaps a peer claim"


@pytest.mark.parametrize(
    ("block_ms", "ok_detours", "critical_detours"),
    [
        (8_000, False, False),  # too short to be worth going round for anyone
        (20_000, True, False),  # the interesting band: charge buys patience
        (60_000, True, True),  # too long for anyone to wait out
    ],
)
def test_energy_state_decides_between_waiting_and_detouring(
    block_ms, ok_detours, critical_detours
):
    graph, ids, cell_of = ladder()
    tbl = table(graph, [blocking(9, ids[(2, 0)], 0, block_ms)])

    def detoured(state: EnergyState) -> bool:
        result = run(graph, tbl, ids[(0, 0)], [ids[(4, 0)]], cost=model(state))
        return any(cell_of[n][1] == 1 for n in result.nodes)

    assert detoured(EnergyState.OK) is ok_detours
    assert detoured(EnergyState.CRITICAL) is critical_detours


def test_a_wait_longer_than_the_budget_is_refused():
    graph = make_graph(line(4))
    tbl = table(graph, [blocking(1, 2, 0, 500_000)])
    result = run(graph, tbl, 0, [3], config=Config(max_wait_ms=1_000))
    assert result.status is PlanStatus.UNREACHABLE


def test_hard_blocked_nodes_are_never_entered():
    graph, ids, _ = ladder()
    result = run(
        graph, table(graph), ids[(0, 0)], [ids[(4, 0)]],
        blocked_nodes=frozenset({ids[(2, 0)]}),
    )
    assert ids[(2, 0)] not in result.nodes


def test_a_soft_penalty_steers_the_route_without_forbidding_it():
    graph, ids, _ = ladder()
    expensive = {ids[(2, 0)]}
    cheap = run(graph, table(graph), ids[(0, 0)], [ids[(4, 0)]])
    steered = run(
        graph, table(graph), ids[(0, 0)], [ids[(4, 0)]],
        node_penalty=lambda n: 50_000.0 if n in expensive else 0.0,
    )
    assert ids[(2, 0)] in cheap.nodes
    assert ids[(2, 0)] not in steered.nodes


def test_search_is_deterministic():
    graph, ids, _ = ladder()
    tbl = table(graph, [blocking(9, ids[(2, 0)], 0, 20_000)])
    runs = [run(graph, tbl, ids[(0, 0)], [ids[(4, 0)]]) for _ in range(5)]
    assert all(r == runs[0] for r in runs)


def test_the_heuristic_never_overestimates_on_the_real_map(real_graph):
    # Admissibility is what makes the exact-distance heuristic safe to use: if it
    # could exceed the true remaining cost, A* would return cheap-looking paths that
    # are not actually cheapest.
    import random

    rng = random.Random(17)
    tbl = ReservationTable(real_graph, [], now=0, config=CFG)
    cost = CostModel.for_state(real_graph.node_spacing, EnergyState.OK)
    floor = cost.min_hop_cost()

    for _ in range(60):
        start, goal = rng.randrange(real_graph.n), rng.randrange(real_graph.n)
        result = search(
            real_graph, tbl, start=start, goals=[goal], cost_model=cost,
            config=CFG, start_heading=Heading.E, moving=True,
        )
        if result.status is not PlanStatus.OK:
            continue
        estimate = real_graph.hops_from(goal)[start] * floor
        assert estimate <= result.cost + 1e-6, (
            f"heuristic {estimate} exceeded realised cost {result.cost}"
        )
