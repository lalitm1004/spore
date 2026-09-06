"""A. Decisions — answering the robot.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

A. Decisions — answering the robot (PROTOCOL.md §16, docs/scenarios.md A)

The robot blocks at every node until it hears back, and if it never does it
sits there for the rest of its shift. So the through-line of this block is
that there is no input, and no internal failure, that makes a bot go quiet.
"""
from __future__ import annotations

import pytest

from tests.containers.harness import (
    _holder_of,
    _kind,
    _map_nodes,
    _neighbours,
    _query,
    _routing_fleet,
    FAST_TIMINGS,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


# -----------------------------------------------------------------------------
@pytest.mark.docker
def test_A1_a_bot_with_no_job_still_answers(one_bot):
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    fleet.place(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    reply = fleet.ask(cs[0], _query(node, _neighbours(node)))
    assert reply is not None, "the bot said nothing at all"
    assert _kind(reply) == "WAIT"
    assert reply.hold_ms > 0, "a zero hold would have it ask in a tight loop"


@pytest.mark.docker
def test_A2_a_bot_given_a_job_is_routed_towards_it(fleet):
    """The whole point: a job becomes turns, one node at a time."""
    start = _map_nodes(PARK, 8)[0]
    cs, holder = _routing_fleet(fleet, "A2", start)
    reply = fleet.ask(holder, _query(start, _neighbours(start)))
    assert _kind(reply) in ("PROCEED", "REROUTE", "WAIT", "YIELD"), reply
    if _kind(reply) in ("PROCEED", "REROUTE"):
        assert reply.target_node_id in _neighbours(start), \
            "it must name a lane the robot said exists"
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_A4_a_changed_route_is_announced_as_a_reroute(fleet):
    """PROCEED and REROUTE differ only in whether the robot's route changed --
    which is what makes a log readable when a bot doubles back."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 12)
    start = nodes[0]

    for c in cs:
        fleet.place(c, latest_node_id=start, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20)
    assert fleet.submit_job(cs[0], "A4-a", nodes[6], nodes[7]).accepted
    holder = _holder_of(fleet, cs, "A4-a")
    assert holder is not None

    first = fleet.ask(holder, _query(start, _neighbours(start)))
    # Same question again with the same route: nothing changed, so nothing is
    # announced as changed.
    second = fleet.ask(holder, _query(start, _neighbours(start), query_id=2))
    assert first is not None and second is not None
    if _kind(second) in ("PROCEED", "REROUTE"):
        assert _kind(second) == "PROCEED", \
            "an unchanged route must not be reported as a reroute"


@pytest.mark.docker
def test_A3_a_bot_standing_on_its_goal_is_told_to_wait(fleet):
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 8)
    goal = nodes[6]

    for c in cs:
        fleet.place(c, latest_node_id=goal, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster),
                      20, what="the roster to catch up")
    ack = fleet.submit_job(cs[0], "A3", goal, nodes[7])
    assert ack.accepted

    holder = _holder_of(fleet, cs, "A3")
    assert holder is not None
    reply = fleet.ask(holder, _query(goal, _neighbours(goal)))
    assert _kind(reply) == "WAIT", reply
    assert reply.hold_ms > 0


@pytest.mark.docker
def test_A5_a_query_offering_turns_we_did_not_plan_is_still_answered(one_bot):
    """Our map and the robot's can disagree. A wrong turn is recoverable at the
    next node; silence is not recoverable at all."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    reply = fleet.ask(cs[0], _query(node, (999999,)))
    assert reply is not None, "a disagreement must not silence the bot"
    assert _kind(reply) in ("WAIT", "PROCEED", "REROUTE", "YIELD")


@pytest.mark.docker
def test_A6_a_message_we_cannot_use_does_not_break_the_link(one_bot):
    """One bad message must not cost the robot the rest of its shift.

    A typed wire moves this failure up a layer rather than removing it. There
    is no longer such a thing as a malformed line -- gRPC rejects that at the
    transport -- but there is still a message that parses and makes no sense:
    a node this map has never heard of, offering exits to nodes that do not
    exist. The stream has to survive it and answer the next question.
    """
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    replies = fleet.converse(cs[0], [
        _query(999999, (999998,), query_id=6),
        _query(node, _neighbours(node), query_id=7),
    ])
    assert len(replies) == 2, "the stream died on the message it could not use"
    assert replies[-1].query_id == 7


@pytest.mark.docker
def test_A7_a_node_this_map_has_never_heard_of_is_still_answered(one_bot):
    fleet, cs = one_bot
    reply = fleet.ask(cs[0], _query(999999, (999998,)))
    assert reply is not None
    assert _kind(reply) in ("WAIT", "PROCEED", "REROUTE", "YIELD")


@pytest.mark.docker
def test_A8_a_bot_with_no_map_answers_and_still_leads(fleet):
    """Geography-blind is a degraded fleet, not a dead one."""
    cs = fleet.launch(1, PARK, WAREHOUSE_MAP="/nonexistent/warehouse.json", **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge without a map")

    reply = fleet.ask(cs[0], _query(434, (435,)))
    assert reply is not None
    assert _kind(reply) == "WAIT"
    assert "map" in reply.because, reply


@pytest.mark.docker
def test_A9_the_query_id_comes_back_exactly(one_bot):
    """Two junctions can share a destination, so the id is the only way the
    robot can tell a fresh answer from a late one."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    for query_id in (1, 7, 4242):
        assert fleet.ask(cs[0], _query(node, _neighbours(node), query_id=query_id)).query_id == query_id


@pytest.mark.docker
def test_A10_one_connection_serves_a_whole_shift(one_bot):
    """A socket per question would be pure overhead on hardware that has none
    to spare, so the companion connects once and keeps it."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    lanes = _neighbours(node)
    replies = fleet.converse(
        cs[0], [_query(node, lanes, query_id=i + 1) for i in range(50)])

    assert len(replies) == 50
    assert [r.query_id for r in replies] == list(range(1, 51)), \
        "answers came back out of order, or one went missing"
