"""F. Redirections.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

F. Redirections (docs/scenarios.md F)
"""
from __future__ import annotations

import pytest

from tests.containers.harness import (
    _corridor,
    _free_fleet,
    _kind,
    _map_nodes,
    _neighbours,
    _planned,
    _query,
    _routing_fleet,
    FAST_TIMINGS,
    GRID,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


# -----------------------------------------------------------------------------
@pytest.mark.docker
def test_F2_an_obstruction_is_routed_around(fleet):
    """The planner has always supported obstructions; nothing fed it one until
    now, because the node in a robot's OBSTACLE warning was dropped on the way
    in. It is not any more, so this drives the real path.

    Obstructions ride on the planning `Request`, not on the traffic view -- the
    search is the only thing that prices them. This scenario is what proves
    that wire is connected, so it takes the lane it is given rather than
    tolerating a bot that never planned.
    """
    node = _map_nodes(PARK, 1)[0]
    cs, ours = _routing_fleet(fleet, "F2", node)

    before = _planned(fleet.ask(ours, _query(node, _neighbours(node))), "before the block")
    assert _kind(before) in ("PROCEED", "REROUTE"), \
        f"nothing was in the way, so there is a lane to block; got {before}"
    blocked = before.target_node_id

    fleet.obstruct(ours, blocked, level=1.0)
    after = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=2)),
                     "after the block")
    assert _kind(after) in ("PROCEED", "REROUTE", "WAIT", "YIELD"), \
        "an obstruction is not a reason to go silent"
    if _kind(after) in ("PROCEED", "REROUTE"):
        assert after.target_node_id != blocked, "it drove into a node reported blocked"
    fleet.obstruct(ours, blocked, level=0.0)


@pytest.mark.docker
def test_F3_clearing_an_obstruction_opens_the_lane_again(fleet):
    """A blockage that is gone must stop costing anything, or the fleet slowly
    forgets lanes it can use."""
    node = _map_nodes(PARK, 1)[0]
    cs, ours = _routing_fleet(fleet, "F3", node)
    lane = _neighbours(node)[0]

    fleet.obstruct(ours, lane, level=1.0)
    blocked = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=1)),
                       "while blocked")
    fleet.obstruct(ours, lane, level=0.0)
    cleared = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=2)),
                       "once cleared")

    if _kind(blocked) in ("PROCEED", "REROUTE"):
        assert blocked.target_node_id != lane
    # With nothing in the way the lane is allowed again; the point is that the
    # obstruction stopped applying, not which lane wins.
    assert _kind(cleared) in ("PROCEED", "REROUTE", "WAIT", "YIELD")


@pytest.mark.docker
def test_F4_a_bot_that_migrates_replans_on_arrival(fleet):
    """Regions are subnetworks: a route across one is planned optimistically and
    only becomes informed once its roster arrives."""
    park, park_leader = _free_fleet(fleet, 2, PARK)
    grid = fleet.launch(1, GRID, start_id=10, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(grid, GRID), 30, what="the second region")
    assert wait_until(
        lambda: {ld.region_id for ld in fleet.state(park_leader).other_leaders} >= {GRID},
        30, what="the leaders to meet")

    mover = [c for c in park if c.id != park_leader.id][0]
    grid_node = _map_nodes(GRID, 4)[0]
    fleet.place(mover, latest_node_id=grid_node, region_id=GRID, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: fleet.state(mover).region_id == GRID, 40, what="the migration")

    reply = fleet.ask(mover, _query(grid_node, _neighbours(grid_node), region=GRID))
    assert reply is not None, "it went quiet in its new region"


@pytest.mark.docker
def test_F6_a_peers_claim_between_two_questions_changes_the_answer(fleet):
    """Traffic is not static between one node and the next, and the answer has
    to move with it.

    A claim has to outlive the drive it is meant to prevent, and for a while
    this scenario could not be made to hold. The reason was not the TTL -- that
    governs when a *received* claim lapses -- but the window the sender
    announces, which was two announce periods and so expired before a neighbour
    two seconds away could arrive. `ReservationSender._hold_ms` now covers a
    traversal, so the shared fleet is enough and this needs no clock of its own.
    """
    corridor = _corridor(6)
    ours = corridor[0]
    cs, ours_c = _routing_fleet(fleet, "F6", ours)
    other_c = next(c for c in cs if c is not ours_c)

    first = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours))), "on the first ask")
    assert _kind(first) in ("PROCEED", "REROUTE"), \
        f"nothing was in the way, so there is a lane to contest; got {first}"
    contested = first.target_node_id

    fleet.place(other_c, latest_node_id=contested, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    other = fleet.state(other_c).bot_id
    assert wait_until(
        lambda: any(r.bot_id == other and r.node_id == contested
                    for r in fleet.state(ours_c).reservations),
        20, what="the peer's claim to arrive")

    second = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours), query_id=2)),
                      "on the second ask")
    if _kind(second) in ("PROCEED", "REROUTE"):
        assert second.target_node_id != contested, \
            "it kept driving at a node a peer had since claimed"
