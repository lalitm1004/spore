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


# ---- head-on ------------------------------------------------------------------

def _corridor_step(bot):
    """A node inside a corridor, and the lane on out of it.

    A head-on needs a corridor: somewhere with no way round, which is exactly
    where two robots meeting cannot recover.
    """
    graph = bot.graph
    for index in range(graph.n):
        if graph.degree(index) != 2:
            continue
        onward = [v for v, _ in graph.neighbours(index)]
        if len(onward) == 2:
            return graph.id_of(index), tuple(graph.id_of(v) for v in onward)
    pytest.skip("this map has no corridor")


def _oncoming(bot, node_id, heading, bot_id=99, rank=2):
    """A peer holding `node_id` and travelling `heading` -- what
    `traffic.predict` emits for a peer whose trail gives it a direction."""
    from planning.types import Reservation
    return botmod.traffic_module.Observation(
        bot_id=bot_id, node_id=node_id, trail=(node_id,),
        reservations=(Reservation(node_id=node_id, t_in=0, t_out=10_000, dir=heading),),
        rank=rank,
    )


def test_a_robot_outranked_in_a_head_on_gives_way(router, monkeypatch):
    """The conflict per-node reservation cannot see.

    A claim reserves a node and says nothing about the lane leading to it, so
    two robots one lane apart -- each holding its own node, each legitimately
    reserved -- can drive into the lane between them from opposite ends. On a
    single painted line there is no passing. Measured on an eight-robot run,
    two pairs ended nose to nose at 0.66 m and 0.69 m, both 180 degrees facing,
    with no claim violated by either.

    The planner already names them (`Diagnostics.corridor_opposing_peers`) and
    deliberately does not act. Acting is this layer's job, and the answer has to
    be asymmetric: if both robots refuse the lane that is a livelock, not a fix.
    """
    node, lanes = _corridor_step(router)
    router.nav_goal = Goal.node(_far_goal(router, node))
    router.obstructions.clear()

    free = router._route(Query(query_id=1, node_id=node, available=lanes))
    if free.kind.value not in ("PROCEED", "REROUTE"):
        pytest.skip("no clear lane here to contest")

    # Someone senior coming the other way down the lane we were about to take.
    from planning.geometry import heading_between
    graph = router.graph
    ours = heading_between(graph.position[graph.index(node)],
                           graph.position[graph.index(free.target_node_id)])
    monkeypatch.setattr(router, "_observations",
                        lambda: (_oncoming(router, free.target_node_id, ours.opposite),))

    reply = router._route(Query(query_id=2, node_id=node, available=lanes))

    assert reply.kind.value in ("YIELD", "WAIT"), \
        "drove into a robot coming the other way"
    assert "head-on" in reply.because


def test_the_robot_with_right_of_way_keeps_going(router, monkeypatch):
    """The other half, and the reason the ordering matters: exactly one of the
    two gives way. A rule that stopped both would trade a deadlock for a
    livelock."""
    node, lanes = _corridor_step(router)
    router.nav_goal = Goal.node(_far_goal(router, node))
    router.obstructions.clear()

    free = router._route(Query(query_id=1, node_id=node, available=lanes))
    if free.kind.value not in ("PROCEED", "REROUTE"):
        pytest.skip("no clear lane here to contest")

    from planning.geometry import heading_between
    graph = router.graph
    ours = heading_between(graph.position[graph.index(node)],
                           graph.position[graph.index(free.target_node_id)])
    # rank 0 and a higher bot id: outranked by us on both counts.
    monkeypatch.setattr(router, "_observations",
                        lambda: (_oncoming(router, free.target_node_id, ours.opposite,
                                           bot_id=99, rank=0),))
    router.cargo_state = "EN_ROUTE"      # we are carrying, so we outrank

    reply = router._route(Query(query_id=2, node_id=node, available=lanes))

    assert reply.kind.value in ("PROCEED", "REROUTE"), \
        "gave way when we had right of way -- both stopping is a livelock"


def test_a_declared_claim_carries_the_peers_direction(router):
    """The gap that made the head-on rule unreachable.

    `Reservation.dir` was set in exactly one place -- `traffic.predict` -- and
    predict returns nothing at all for a peer that has declared any claim,
    because tier 1 beats tier 2. Every moving robot declares claims, so `dir`
    was never set for the robots that matter, `corridor_opposing_peers` was
    always empty, and the rule that reads it could not fire. Measured with the
    rule in place: four of five robots funnelled onto one corridor and two
    ended 1.07 m apart facing each other, in silence.

    The heading is not new information -- it is two entries of the peer's own
    trail, already in every `PeerRecord`.
    """
    from peers.table import Peer
    graph = router.graph
    # Two adjacent nodes give a heading; claim the one the peer is standing on.
    here = next(i for i in range(graph.n) if graph.degree(i) >= 1)
    onward = next(v for v, _ in graph.neighbours(here))
    peer_id = 77

    router.peer_table.upsert(Peer(
        bot_id=peer_id, address="x:1", priority=1, state="IDLE", battery=100.0,
        latest_node_id=graph.id_of(onward),
        node_trail=[graph.id_of(onward), graph.id_of(here)],
    ))
    from reservations.claims import Announce, Window
    router.ledger.receive(
        Announce(bot_id=peer_id, seq=1, windows=(
            Window(node_id=graph.id_of(onward), start_offset_ms=0, end_offset_ms=10_000),)),
        now=0)

    seen = [o for o in router._observations() if o.bot_id == peer_id]
    assert seen, "the peer never reached the observations at all"
    dirs = [r.dir for r in seen[0].reservations]
    assert dirs and all(d is not None for d in dirs), \
        "a declared claim carries no direction, so no head-on can ever be seen"
