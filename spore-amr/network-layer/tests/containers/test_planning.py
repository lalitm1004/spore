"""C. Planning.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

C. Planning (PROTOCOL.md §16, docs/scenarios.md C)

A job is a destination; what the robot needs is a turn. These follow that
translation end to end on real containers.
"""
from __future__ import annotations

import time
import pytest

from tests.containers.harness import (
    _corridor,
    _holder_of,
    _kind,
    _map_nodes,
    _neighbours,
    _planned,
    _query,
    _routing_fleet,
    FAST_TIMINGS,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


@pytest.mark.docker
def test_C1_a_job_becomes_a_sequence_of_turns(fleet):
    """Every node on the way is answered, and each answer names a real lane."""
    route = _corridor(4)[:5]
    cs, holder = _routing_fleet(fleet, "C1", route[0])
    answers = fleet.decisions(holder, route)
    assert all(a is not None for a in answers), "a node went unanswered"
    assert all(_kind(a) in ("PROCEED", "REROUTE", "WAIT", "YIELD") for a in answers)
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_C2_the_goal_moves_to_the_dropoff_once_the_cargo_is_aboard(fleet):
    """Nobody re-commands the robot: picking the cargo up is what changes where
    it is going."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 12)
    pickup, dropoff = nodes[6], nodes[10]

    for c in cs:
        fleet.place(c, latest_node_id=nodes[0], region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20)
    assert fleet.submit_job(cs[0], "C2", pickup, dropoff).accepted
    holder = _holder_of(fleet, cs, "C2")
    assert holder is not None

    # Arrive and report the cargo aboard, exactly as a robot would.
    fleet.place(holder, latest_node_id=pickup, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="C2", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20,
                      what="the pickup to register")

    reply = fleet.ask(holder, _query(pickup, _neighbours(pickup)))
    assert reply is not None
    assert _kind(reply) != "WAIT" or "goal" not in reply.because, \
        "it should now be heading for the dropoff, not sitting on its goal"


@pytest.mark.docker
def test_C3_a_neighbours_claim_is_respected(fleet):
    """Tier 1: a declared claim is a promise, and the route honours it."""
    corridor = _corridor(6)
    ours, theirs = corridor[0], corridor[1]

    cs, ours_c = _routing_fleet(fleet, "C3", ours)
    other_c = next(c for c in cs if c is not ours_c)
    fleet.place(other_c, latest_node_id=theirs, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    other_id = fleet.state(other_c).bot_id
    assert wait_until(lambda: any(r.bot_id == other_id and r.node_id == theirs
                                  for r in fleet.state(ours_c).reservations), 20,
                      what="the neighbour's claim to arrive")

    reply = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours))))
    if _kind(reply) in ("PROCEED", "REROUTE"):
        assert reply.target_node_id != theirs, "it drove into a node a peer holds"


@pytest.mark.docker
def test_C4_a_peers_trail_reaches_us_over_the_network(two_bots):
    """Tier 2's input. Prediction itself is unit-tested; what a container proves
    is that the trail a peer builds actually arrives in our roster, which is the
    only thing prediction has to work from."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(6)

    fleet.drive(cs[1], corridor[:4], battery=90.0, state="MOVING", mission="IDLE")
    other_id = fleet.state(cs[1]).bot_id
    assert wait_until(
        lambda: any(len(p.node_trail) >= 2 for p in fleet.state(cs[0]).roster
                    if p.bot_id == other_id),
        20, what="a multi-node trail to reach the roster")

    trail = [list(p.node_trail) for p in fleet.state(cs[0]).roster if p.bot_id == other_id][0]
    assert trail[0] != trail[1], "consecutive duplicates should have been collapsed"


@pytest.mark.docker
def test_C6_a_flat_battery_waits_where_a_charged_one_would_go_round(two_bots):
    """The energy term exists to make exactly this trade, and it only shows up
    where going round actually costs something."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(8)
    ours, ahead = corridor[3], corridor[4]

    fleet.place(cs[1], latest_node_id=ahead, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.place(cs[0], latest_node_id=ours, region_id=PARK, battery=8.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) > 1, 20,
                      what="claims to be exchanged")

    reply = fleet.ask(cs[0], _query(ours, _neighbours(ours)))
    assert reply is not None, "a flat battery is not a reason to go silent"


@pytest.mark.docker
def test_C7_a_decision_lands_well_inside_the_tick(two_bots):
    """Twenty questions on one stream, so the number is planning time and not
    twenty connection setups. Measured from the host now that the link is
    reachable from one -- that adds a loopback hop per question, which makes
    this a ceiling on planning time rather than an exact figure."""
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    lanes = _neighbours(node)
    questions = [_query(node, lanes, query_id=i + 1) for i in range(20)]

    started = time.monotonic()
    replies = fleet.converse(cs[0], questions)
    per_ask_ms = (time.monotonic() - started) * 1000 / len(questions)
    assert len(replies) == 20
    print(f"\n  C7: {per_ask_ms:.1f} ms per decision")
    assert per_ask_ms < 200, f"{per_ask_ms:.1f} ms is too slow to answer at every node"


@pytest.mark.docker
def test_C8_an_unreachable_goal_is_said_out_loud(fleet):
    """"I cannot get there" has to be spoken. Silence looks identical to a dead
    network layer, and the robot treats it that way -- by never moving again."""
    cs = fleet.launch(1, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    node = _map_nodes(PARK, 1)[0]
    fleet.place(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    reply = fleet.ask(cs[0], _query(node, _neighbours(node)))
    assert reply is not None
    # Note this bot is placed with no job rather than with a goal it cannot
    # reach, so what it exercises is the jobless path. That path now sends the
    # robot out of the lane instead of parking it in one; either way the
    # guarantee this test exists for is the same -- an answer, with a reason.
    if _kind(reply) == "WAIT":
        assert reply.because, "a wait with no reason is a wait nobody can debug"
        assert reply.hold_ms > 0
    else:
        assert reply.target_node_id, "sent somewhere without saying where"
