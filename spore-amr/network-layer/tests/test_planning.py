"""End-to-end planner behaviour."""

from __future__ import annotations

from itertools import pairwise

from tests.planning_maps import make_graph, make_map_doc

from planning import (
    Config,
    EnergyState,
    Goal,
    Obstruction,
    PeerView,
    Planner,
    PlanStatus,
    RegionGossip,
    Request,
    Reservation,
    SelfState,
)
from planning.kinematics import DEFAULT_KINEMATICS
from planning import Graph, NodeType
from warehouse.map import WarehouseMap
from planning.geometry import Heading

TRAVERSE_MS = DEFAULT_KINEMATICS.cruise_ms(200)


def ladder_planner():
    cells = {(x, y): "PT" for x in range(6) for y in range(2)}
    cells[(5, 1)] = "CH"
    graph = make_graph(cells)
    ids = {cell: i for i, cell in enumerate(sorted(cells))}
    return Planner(graph, bot_id=1), graph, ids


def ask(planner, node_id, goal, **kwargs):
    state = SelfState(
        node_id=node_id,
        heading=kwargs.pop("heading", Heading.E),
        moving=kwargs.pop("moving", False),
        energy=kwargs.pop("energy", EnergyState.OK),
        urgent=kwargs.pop("urgent", False),
    )
    return planner.plan(
        Request(now=kwargs.pop("now", 0), self_state=state, goal=goal, **kwargs)
    )


# -- the reservation windows -------------------------------------------------


def test_a_plan_reaches_its_goal_on_the_real_map(real_graph):
    planner = Planner(real_graph, bot_id=1)
    result = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(400)))
    assert result.status is PlanStatus.OK
    assert result.path.hops[0].node_id == real_graph.id_of(0)
    assert result.path.goal_node_id == real_graph.id_of(400)
    assert result.changed


def test_consecutive_claims_overlap_by_exactly_one_traversal(real_graph):
    # The invariant node-only reservations rest on: a robot holds a node until it is
    # fully inside the next, so a head-on swap shows up as an overlap on a node.
    planner = Planner(real_graph, bot_id=1)
    hops = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(300))).path.hops
    for this, nxt in pairwise(hops):
        assert this.t_out >= nxt.t_in
        assert this.t_out - nxt.t_in == TRAVERSE_MS


def test_windows_never_run_backwards(real_graph):
    planner = Planner(real_graph, bot_id=1)
    for hop in ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(250))).path.hops:
        assert hop.t_out >= hop.t_in


def test_only_the_first_k_commit_hops_are_committed(real_graph):
    planner = Planner(real_graph, bot_id=1, config=Config(k_commit=5))
    path = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(400))).path
    assert len(path.hops) > 5
    assert path.committed == 5


def test_each_hop_direction_matches_the_step_it_describes(real_graph):
    planner = Planner(real_graph, bot_id=1)
    path = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(200))).path
    for i, hop in enumerate(path.hops[:-1]):
        step = real_graph.heading_to(
            real_graph.index(hop.node_id), real_graph.index(path.hops[i + 1].node_id)
        )
        assert hop.dir is step
    assert path.hops[-1].dir is None, "the last hop has not committed to a direction"


# -- outcomes ----------------------------------------------------------------


def test_planning_to_where_the_robot_already_stands(real_graph):
    planner = Planner(real_graph, bot_id=1)
    node = real_graph.id_of(10)
    result = ask(planner, node, Goal.node(node))
    assert result.status is PlanStatus.ALREADY_THERE
    assert result.path.hops[0].node_id == node


def test_a_goal_that_is_not_on_the_map_is_rejected(real_graph):
    planner = Planner(real_graph, bot_id=1)
    result = ask(planner, real_graph.id_of(0), Goal.node(10**9))
    assert result.status is PlanStatus.NO_GOAL_AVAILABLE
    assert result.path is None


def test_a_robot_that_is_not_on_the_map_is_rejected(real_graph):
    planner = Planner(real_graph, bot_id=1)
    assert ask(planner, 10**9, Goal.node(real_graph.id_of(0))).status is PlanStatus.UNREACHABLE


def test_a_peer_on_our_own_node_reports_start_blocked(real_graph):
    planner = Planner(real_graph, bot_id=1)
    node = real_graph.id_of(100)
    peer = PeerView(bot_id=2, reservations=(Reservation(node_id=node, t_in=0, t_out=9000),))
    result = ask(planner, node, Goal.node(real_graph.id_of(200)), peers=(peer,))
    assert result.status is PlanStatus.START_BLOCKED


# -- class goals -------------------------------------------------------------


def test_charge_and_park_resolve_to_the_right_kind_of_node(real_graph):
    planner = Planner(real_graph, bot_id=1)
    for goal, node_type in [(Goal.charge(), NodeType.CH), (Goal.park(), NodeType.PK)]:
        result = ask(planner, real_graph.id_of(0), goal)
        assert result.status is PlanStatus.OK
        chosen = real_graph.index(result.path.goal_node_id)
        assert real_graph.node_type[chosen] is node_type


def test_a_charger_a_peer_is_heading_for_is_avoided():
    # Two chargers, one near and one far. A peer whose committed path ends on the
    # near one is going there, so it should not be this robot's first choice.
    cells = {(x, y) for x in range(6) for y in range(2)}
    graph = make_graph({c: ("CH" if c in {(0, 1), (5, 1)} else "PT") for c in cells})
    ids = {cell: i for i, cell in enumerate(sorted(cells))}
    planner = Planner(graph, bot_id=1)
    start = graph.id_of(ids[(1, 0)])
    near = graph.id_of(ids[(0, 1)])

    free = ask(planner, start, Goal.charge())
    assert free.path.goal_node_id == near, "unclaimed, the near charger is the obvious pick"

    peer = PeerView(
        bot_id=2, reservations=(Reservation(node_id=near, t_in=50_000, t_out=60_000),)
    )
    taken = ask(planner, start, Goal.charge(), peers=(peer,))
    assert taken.path.goal_node_id != near, "claimed, it should go elsewhere"
    assert "1 already targeted by peers" in taken.diagnostics.goal_rationale


def test_gossip_that_a_region_is_full_discriminates_when_chargers_span_regions():
    # Two chargers in two regions: the gossip has something to choose between.
    doc = make_map_doc({(x, 0): ("CH" if x in (0, 6) else "PT") for x in range(7)})
    doc["regions"].append(
        {"id": 2, "name": "far", "density": "sparse", "description": "far bank"}
    )
    for node in doc["nodes"]:
        if node["position"]["x"] >= 600.0:
            node["region_id"] = 2
    graph = Graph(WarehouseMap(doc))
    planner = Planner(graph, bot_id=1)
    start = graph.id_of(2)

    plain = ask(planner, start, Goal.charge())
    assert plain.path.goal_node_id == graph.id_of(0), "the near charger is the obvious pick"

    near_region = graph.region_id[graph.index(plain.path.goal_node_id)]
    steered = ask(
        planner, start, Goal.charge(),
        gossip=(RegionGossip(region_id=near_region, chargers_free=0),),
    )
    assert steered.path.goal_node_id == graph.id_of(6), "told it is full, it goes to the other"


def test_region_gossip_cannot_pick_between_chargers_that_share_a_region(real_graph):
    # A property of this floor plan, not of the planner: all 34 CH nodes live in the
    # single `charging` region, so a per-region `chargers_free` report prices every
    # candidate identically and cannot break the tie. It still raises the cost of
    # charging overall, which is visible; what discriminates between individual
    # chargers on this map is peer claims and per-node load, not region gossip.
    planner = Planner(real_graph, bot_id=1)
    start = real_graph.id_of(0)
    plain = ask(planner, start, Goal.charge())
    region = real_graph.region_id[real_graph.index(plain.path.goal_node_id)]
    assert {real_graph.region_id[n] for n in real_graph.nodes_of_type(NodeType.CH)} == {region}

    steered = ask(
        planner, start, Goal.charge(),
        gossip=(RegionGossip(region_id=region, chargers_free=0),),
    )
    assert steered.status is PlanStatus.OK
    assert steered.path.goal_node_id == plain.path.goal_node_id
    assert steered.path.cost > plain.path.cost


def test_a_map_with_no_charger_reports_no_goal_available():
    graph = make_graph({(x, 0): "PT" for x in range(4)})
    planner = Planner(graph, bot_id=1)
    result = ask(planner, graph.id_of(0), Goal.charge())
    assert result.status is PlanStatus.NO_GOAL_AVAILABLE
    assert "no CH nodes" in result.diagnostics.goal_rationale


# -- obstructions ------------------------------------------------------------


def test_a_severe_obstruction_is_impassable():
    planner, graph, ids = ladder_planner()
    blocked = graph.id_of(ids[(2, 0)])
    result = ask(
        planner,
        graph.id_of(ids[(0, 0)]),
        Goal.node(graph.id_of(ids[(4, 0)])),
        obstructions=(Obstruction(node_id=blocked, level=1.0),),
    )
    assert blocked not in result.path.node_ids


def test_a_mild_obstruction_only_makes_a_node_expensive():
    planner, graph, ids = ladder_planner()
    node = graph.id_of(ids[(2, 0)])
    mild = ask(
        planner,
        graph.id_of(ids[(0, 0)]),
        Goal.node(graph.id_of(ids[(4, 0)])),
        obstructions=(Obstruction(node_id=node, level=0.1),),
    )
    # Cheap enough to be worth pushing through rather than turning twice to avoid.
    assert node in mild.path.node_ids


# -- hysteresis --------------------------------------------------------------


def test_an_unchanged_world_does_not_produce_a_new_path(real_graph):
    planner = Planner(real_graph, bot_id=1)
    start, goal = real_graph.id_of(0), real_graph.id_of(200)
    first = ask(planner, start, Goal.node(goal))
    again = ask(planner, start, Goal.node(goal), current=first.path)
    assert not again.changed
    assert again.path == first.path
    assert "still competitive" in again.diagnostics.replan_reason


def test_a_robot_part_way_along_its_path_keeps_following_it(real_graph):
    # Advancing is not a reason to replan -- if it were, hysteresis would never bite
    # and the robot would rebuild its route at every node.
    planner = Planner(real_graph, bot_id=1)
    first = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(200)))
    moved_to = first.path.hops[3].node_id
    again = ask(planner, moved_to, Goal.node(real_graph.id_of(200)), current=first.path)
    assert not again.changed


def test_a_robot_that_has_left_its_path_replans(real_graph):
    planner = Planner(real_graph, bot_id=1)
    first = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(200)))
    off_path = next(
        real_graph.id_of(i)
        for i in range(real_graph.n)
        if real_graph.id_of(i) not in first.path.node_ids
    )
    again = ask(planner, off_path, Goal.node(real_graph.id_of(200)), current=first.path)
    assert again.changed
    assert "no longer on its planned path" in again.diagnostics.replan_reason


def test_validation_follows_the_robot_along_its_path(real_graph):
    # The committed window that matters is the one *ahead* of the robot. Checking
    # the hops it started with means re-validating road it has already driven, while
    # the nodes it is about to claim go unchecked -- which is how a robot ends up
    # executing a path a peer has since taken.
    planner = Planner(real_graph, bot_id=1, config=Config(k_commit=4))
    first = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(300)))
    hops = first.path.hops
    standing_at = hops[6].node_id

    def claim(hop):
        return PeerView(
            bot_id=9,
            reservations=(
                Reservation(node_id=hop.node_id, t_in=hop.t_in, t_out=hop.t_out),
            ),
        )

    behind = ask(
        planner, standing_at, Goal.node(real_graph.id_of(300)),
        current=first.path, peers=(claim(hops[1]),),
    )
    assert not behind.changed, "a claim on road already travelled is irrelevant"

    ahead = ask(
        planner, standing_at, Goal.node(real_graph.id_of(300)),
        current=first.path, peers=(claim(hops[8]),),
    )
    assert ahead.changed
    assert "claimed by [9]" in ahead.diagnostics.replan_reason


def test_a_class_goal_is_not_invalidated_by_picking_a_different_bay(real_graph):
    # CHARGE does not name a node, so the current path heading for one charger is
    # not "wrong" merely because this tick's search liked another.
    planner = Planner(real_graph, bot_id=1)
    start = real_graph.id_of(0)
    first = ask(planner, start, Goal.charge())
    again = ask(planner, start, Goal.charge(), current=first.path)
    assert not again.changed
    assert "goal changed" not in again.diagnostics.replan_reason


def test_a_path_whose_nodes_have_been_claimed_is_replaced(real_graph):
    planner = Planner(real_graph, bot_id=1)
    start, goal = real_graph.id_of(0), real_graph.id_of(200)
    first = ask(planner, start, Goal.node(goal))
    contested = first.path.hops[2]
    peer = PeerView(
        bot_id=9,
        reservations=(
            Reservation(node_id=contested.node_id, t_in=contested.t_in, t_out=contested.t_out),
        ),
    )
    again = ask(planner, start, Goal.node(goal), current=first.path, peers=(peer,))
    assert again.changed
    assert "claimed by [9]" in again.diagnostics.replan_reason


def test_a_better_path_must_win_for_several_ticks_before_it_is_taken(real_graph):
    planner = Planner(real_graph, bot_id=1, config=Config(stable_ticks=3, k_commit=8))
    start, goal = real_graph.id_of(0), real_graph.id_of(200)
    baseline = ask(planner, start, Goal.node(goal)).path
    # Pretend the path in hand is far worse than anything the planner will find.
    from dataclasses import replace

    expensive = replace(baseline, cost=baseline.cost * 10)
    adopted_on = None
    for tick in range(3):
        result = ask(planner, start, Goal.node(goal), current=expensive, stable_for=tick)
        if result.changed:
            adopted_on = tick
            break
    assert adopted_on == 2, "should hold out for stable_ticks before switching"


# -- yielding and diagnostics ------------------------------------------------


def test_a_long_wait_produces_a_yield_suggestion():
    cells = {(x, 0): "PT" for x in range(8)}
    cells[(3, 1)] = "YI"
    graph = make_graph(cells)
    ids = {c: i for i, c in enumerate(sorted(cells))}
    planner = Planner(graph, bot_id=1)
    peer = PeerView(
        bot_id=4,
        reservations=(Reservation(node_id=graph.id_of(ids[(5, 0)]), t_in=0, t_out=25_000),),
    )
    result = ask(planner, graph.id_of(ids[(0, 0)]), Goal.node(graph.id_of(ids[(7, 0)])), peers=(peer,))
    assert result.yield_to is not None
    assert result.yield_to.kind == "YI"
    assert result.yield_to.node_id == graph.id_of(ids[(3, 1)])
    assert result.diagnostics.blocking_peers == (4,)


def test_with_no_yield_bay_the_cascade_falls_back_to_a_junction():
    cells = {(x, 0): "PT" for x in range(8)}
    cells[(3, 1)] = "PT"  # a plain spur makes (3,0) a junction, but is not a YI bay
    graph = make_graph(cells)
    ids = {c: i for i, c in enumerate(sorted(cells))}
    planner = Planner(graph, bot_id=1)
    peer = PeerView(
        bot_id=4,
        reservations=(Reservation(node_id=graph.id_of(ids[(5, 0)]), t_in=0, t_out=25_000),),
    )
    result = ask(planner, graph.id_of(ids[(0, 0)]), Goal.node(graph.id_of(ids[(7, 0)])), peers=(peer,))
    assert result.yield_to is not None
    assert result.yield_to.kind == "JUNCTION"


def test_a_clear_run_needs_no_yield_suggestion(real_graph):
    planner = Planner(real_graph, bot_id=1)
    assert ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(100))).yield_to is None


def test_the_corridor_being_entered_is_reported_for_the_deadlock_layer(real_graph):
    planner = Planner(real_graph, bot_id=1)
    result = ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(400)))
    corridor = result.diagnostics.corridor_entered
    assert len(corridor) > 1
    assert result.path.hops[0].node_id in corridor
    assert result.path.hops[1].node_id in corridor


def test_a_peer_coming_the_other_way_down_our_corridor_is_named(real_graph):
    planner = Planner(real_graph, bot_id=1)
    start = real_graph.id_of(0)
    goal = real_graph.id_of(400)
    probe = ask(planner, start, Goal.node(goal))
    heading = probe.path.hops[0].dir
    deep_in = probe.diagnostics.corridor_entered[-1]
    peer = PeerView(
        bot_id=6,
        node_id=deep_in,
        reservations=(
            Reservation(node_id=deep_in, t_in=0, t_out=1000, dir=heading.opposite),
        ),
    )
    result = ask(planner, start, Goal.node(goal), peers=(peer,))
    assert 6 in result.diagnostics.corridor_opposing_peers


def test_planning_is_deterministic(real_graph):
    planner = Planner(real_graph, bot_id=1)
    runs = [
        ask(planner, real_graph.id_of(0), Goal.node(real_graph.id_of(300))) for _ in range(4)
    ]
    assert all(r.path == runs[0].path for r in runs)
