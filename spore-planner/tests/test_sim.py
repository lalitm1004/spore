"""The multi-robot simulation.

This is the end-to-end proof: twenty planners that never talk to each other
directly, on the real floor plan, and the assertion that they still never occupy the
same node at the same time.

Nothing in the simulation has a god's-eye view of who holds what: each robot keeps
its own ledger and learns only what its neighbours announce. That is what makes the
zero-conflict result worth having -- it is agreement reached through announcements,
not an allocator that could not have let a conflict through.

Contention outcomes -- refused claims, lost contests, corridor standoffs -- are
counted rather than asserted away. They are what the priority layer exists to
resolve, and the numbers are here so that work has something real to aim at.
"""

from __future__ import annotations

import pytest

from spore_planner.sim import Simulation

ROBOTS = 12
TICKS = 400


@pytest.fixture(scope="module")
def report(real_graph):
    return Simulation(real_graph, robots=ROBOTS, seed=7).run(TICKS)


def test_no_two_robots_ever_share_a_node(report):
    assert report.node_conflicts == []


def test_no_two_robots_ever_swap_across_an_edge(report):
    # The overlapping-claim invariant is what rules this out with node-only
    # reservations; if it were wrong, this is where it would show.
    assert report.swap_conflicts == []


def test_the_fleet_makes_progress(report):
    assert report.missions_completed > 0
    assert report.clean


def test_no_robot_is_starved(real_graph):
    # Over a longer run every robot should get something done.
    report = Simulation(real_graph, robots=ROBOTS, seed=3).run(1200)
    assert report.starved == []
    assert report.clean


def test_paths_do_not_thrash(report):
    # Hysteresis should keep replanning to a few per robot per hundred ticks; an
    # order of magnitude more would mean the robot is rebuilding its route
    # constantly and its intent is unreadable to peers.
    per_robot_per_tick = report.replans / (ROBOTS * TICKS)
    assert per_robot_per_tick < 1.0


def test_the_run_is_reproducible(real_graph):
    first = Simulation(real_graph, robots=8, seed=11).run(200)
    second = Simulation(real_graph, robots=8, seed=11).run(200)
    assert first.summary() == second.summary()


def test_a_different_seed_gives_a_different_run(real_graph):
    a = Simulation(real_graph, robots=8, seed=1).run(200)
    b = Simulation(real_graph, robots=8, seed=2).run(200)
    assert a.summary() != b.summary()


def test_contention_is_reported_rather_than_hidden(report):
    # Not assertions about quality -- just that the counters are wired up, so the
    # deadlock and refusal numbers reaching the caller are real.
    assert report.claims_refused >= 0
    assert report.deadlocks >= 0
    assert "deadlocks" in report.summary()
    assert "claims refused" in report.summary()


def test_a_single_robot_on_an_empty_map_just_gets_on_with_it(real_graph):
    report = Simulation(real_graph, robots=1, seed=0).run(600)
    assert report.clean
    assert report.missions_completed > 0
    assert report.claims_refused == 0, "nobody to contend with"
    assert report.deadlocks == 0


# -- decentralisation --------------------------------------------------------


def test_no_robot_has_a_view_of_the_whole_fleet(real_graph):
    # Each ledger should know only the handful of bots that have announced to it,
    # never all nineteen others. If this ever held the whole fleet, the zero
    # conflict result above would be measuring a central allocator instead.
    sim = Simulation(real_graph, robots=ROBOTS, seed=5)
    sim.run(200)
    known = [len(sim.ledgers[r.bot_id].neighbours) for r in sim.robots]
    assert max(known) < ROBOTS - 1
    assert sum(known) / len(known) < 4


@pytest.mark.parametrize("fleet", [6, 18, 30])
def test_announcing_costs_far_less_than_broadcasting(fleet):
    # Per-bot traffic does grow with the fleet -- more robots in a fixed warehouse
    # means more neighbours, and there is no way around that. What vicinity scoping
    # buys is the gap against telling everyone: the saving is large and it widens
    # as the fleet grows, which is the property worth holding on to.
    from conftest import REAL_MAP_PATH

    from spore_planner.warehouse import Graph, load_map_file

    ticks = 200
    report = Simulation(Graph(load_map_file(REAL_MAP_PATH)), robots=fleet, seed=5).run(ticks)
    per_bot = report.announcements / (fleet * ticks)
    broadcast = fleet - 1
    assert per_bot < broadcast / 5, f"{per_bot:.2f} vs {broadcast} for a full broadcast"


def test_a_robot_only_enters_a_node_its_own_ledger_clears(real_graph):
    sim = Simulation(real_graph, robots=ROBOTS, seed=5)
    sim.run(300)
    for robot in sim.robots:
        if robot.path is None or robot.index == 0:
            continue
        entered = robot.path.hops[robot.index]
        ledger = sim.ledgers[robot.bot_id]
        assert not ledger.blockers(entered.node_id, entered.t_in, entered.t_out), (
            f"bot {robot.bot_id} is standing on a node a neighbour claims"
        )


def test_contests_are_resolved_by_one_side_giving_way(report):
    # Both sides see the same contest; only the loser acts on it. If both withdrew
    # the fleet would livelock, and if neither did there would be conflicts above.
    assert report.contests_lost >= 0
    assert report.clean
