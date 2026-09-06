"""The router end to end, in process: query in, decision out.

WHAT
    `Bot._route` -- the callable `planning.robot_service` hands every question
    to. It builds the traffic view, calls the planner and turns the plan into
    one `Decision`.

WHERE
    Between `tests/test_planning*.py`, which exercise the search on synthetic
    maps, and `tests/containers/`, which exercises whole containers. Nothing
    used to sit here, and that gap had a cost: the router built a traffic view
    carrying obstructions and then handed the planner a `Request` without them,
    so a robot would drive into a node it had just been told was impassable.
    The Docker scenario meant to catch it (F2) asked a bot that held no job, so
    the reply was `WAIT "no job"` and the assertion behind
    `if kind in ("PROCEED", "REROUTE")` never ran.

WHY
    Both halves of that failure are cheap to prevent here. These tests need no
    daemon and no containers, so they run on every change rather than only when
    Docker is up.
"""
from __future__ import annotations

import pytest

import bot as botmod
from planning.decide import Query
from planning.types import Goal


@pytest.fixture(scope="module")
def router():
    """One bot, real map, no threads started -- `_route` is pure enough to call."""
    return botmod.Bot()


def _junction(bot, min_degree: int = 2) -> tuple[int, tuple[int, ...]]:
    """A node with a real choice of lanes, and the nodes those lanes reach.

    A node with one way out cannot show a detour. The lanes are node ids, not
    turn names, because that is what a robot offers: it holds the map and knows
    the heading it arrived on, so what is reachable is its call.
    """
    graph = bot.graph
    for index in range(graph.n):
        lanes = tuple(graph.id_of(v) for v, _ in graph.neighbours(index))
        if len(lanes) >= min_degree:
            return graph.id_of(index), lanes
    raise AssertionError("no node on this map offers a choice")


def _far_goal(bot, start: int) -> int:
    """Somewhere far enough that the route has to commit to a direction."""
    graph = bot.graph
    best, best_hops = None, -1
    for index in range(0, graph.n, 7):
        node = graph.id_of(index)
        hops = bot.graph.map.distance(start, node)
        if hops != float("inf") and hops > best_hops:
            best, best_hops = node, hops
    assert best is not None, "nothing is reachable from the start node"
    return best


def test_a_bot_with_no_job_waits_rather_than_going_silent(router):
    """The precondition every other test here depends on, stated once.

    It is also the guarantee itself: a robot is never answered with silence.
    """
    router.nav_goal = None
    node, lanes = _junction(router)
    reply = router._route(Query(query_id=1, node_id=node, available=lanes))
    assert reply.kind.value == "WAIT"
    assert reply.because == "no job"
    assert reply.hold_ms > 0


def test_the_query_id_comes_back_untouched(router):
    node, lanes = _junction(router)
    router.nav_goal = Goal.node(_far_goal(router, node))
    assert router._route(Query(query_id=4242, node_id=node, available=lanes)).query_id == 4242


def test_an_obstruction_reaches_the_search(router):
    """The regression. Tier 3 travels on the `Request`, because the search is
    the only thing that prices it -- a blocked node is one the planner declines
    to route through, and it can only decline what it was told about.
    """
    node, lanes = _junction(router)
    router.obstructions.clear()
    router.nav_goal = Goal.node(_far_goal(router, node))

    before = router._route(Query(query_id=1, node_id=node, available=lanes))
    assert before.kind.value in ("PROCEED", "REROUTE"), \
        "nothing is in the way, so there should be a lane to take"
    blocked = before.target_node_id

    router.set_obstruction(blocked, 1.0)
    after = router._route(Query(query_id=2, node_id=node, available=lanes))
    if after.kind.value in ("PROCEED", "REROUTE"):
        assert after.target_node_id != blocked, \
            "it drove into the node it had been told was impassable"
    router.obstructions.clear()


def test_clearing_an_obstruction_gives_the_lane_back(router):
    """A blockage that is gone must stop costing anything, or the fleet slowly
    forgets lanes it can still use."""
    node, lanes = _junction(router)
    router.obstructions.clear()
    router.nav_goal = Goal.node(_far_goal(router, node))

    original = router._route(Query(query_id=1, node_id=node, available=lanes))
    assert original.kind.value in ("PROCEED", "REROUTE")

    router.set_obstruction(original.target_node_id, 1.0)
    router._route(Query(query_id=2, node_id=node, available=lanes))
    router.set_obstruction(original.target_node_id, 0.0)
    assert not router.obstructions

    restored = router._route(Query(query_id=3, node_id=node, available=lanes))
    assert restored.kind.value in ("PROCEED", "REROUTE")
    assert restored.target_node_id == original.target_node_id, \
        "the cheapest lane did not come back once nothing was in the way"


def test_a_goal_off_the_map_waits_with_a_reason(router):
    """Never silence, even when the answer is that there is no answer."""
    node, lanes = _junction(router)
    router.nav_goal = Goal.node(10**9)
    reply = router._route(Query(query_id=1, node_id=node, available=lanes))
    assert reply.kind.value == "WAIT"
    assert reply.because, "a wait the robot cannot explain is a wait nobody can debug"
