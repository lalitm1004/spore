"""The stand-in network layer: its wire format and its routing.

Both halves matter. The wire format is the boundary the real TypeScript layer
will inherit, so it is tested as a contract; the routing is a mock, so it is
tested only for the properties a robot depends on -- legality, reproducibility,
and answering the question it was actually asked.
"""

import json
import math

import pytest

from robot.network import Decision, Query, RandomRouter


def query(available, query_id=1, node_id=5, heading=0.0):
    return Query(query_id=query_id, node_id=node_id, node_type="PT",
                 region_id=2, x_cm=100.0, y_cm=200.0, heading_rad=heading,
                 available=available)


# ------------------------------------------------------------ the contract --

def test_query_round_trips_over_the_wire():
    original = query({"left": 9, "straight": 6}, query_id=42, heading=1.57)
    restored = Query.from_json(original.to_json())

    assert restored.query_id == 42
    assert restored.node_id == 5
    assert restored.available == {"left": 9, "straight": 6}
    assert restored.heading_rad == pytest.approx(1.57, abs=1e-4)


def test_decision_round_trips_over_the_wire():
    restored = Decision.from_json(
        Decision(query_id=7, turn="left", target_node_id=9).to_json())

    assert (restored.query_id, restored.turn, restored.target_node_id) == (7, "left", 9)


def test_wire_format_is_json_lines():
    """The transport splits on newlines, so a payload must not contain one."""
    for text in (query({"left": 9}).to_json(),
                 Decision(query_id=1, turn="left", target_node_id=9).to_json()):
        assert "\n" not in text
        json.loads(text)


def test_an_unknown_turn_is_rejected():
    with pytest.raises(ValueError, match="unknown turn"):
        Decision.from_json('{"query_id":1,"turn":"reverse","target_node_id":2}')


def test_a_query_with_no_available_turns_round_trips():
    """A dead end is a legal thing to be asked about."""
    assert Query.from_json(query({}).to_json()).available == {}


# -------------------------------------------------------------- the router --

def test_it_only_ever_returns_a_turn_that_was_offered():
    """The property the whole design rests on: whatever comes back, the robot
    can drive it. A blind pick from left/straight/right would name walls."""
    router = RandomRouter(seed=7)
    available = {"left": 9, "straight": 6}

    for index in range(200):
        decision = router.route(query(available, query_id=index))
        assert decision.turn in available
        assert decision.target_node_id == available[decision.turn]


def test_it_echoes_the_query_id():
    """Without it, a late answer to the previous junction is indistinguishable
    from the answer to this one -- and two junctions can share a target."""
    assert RandomRouter(seed=1).route(query({"left": 9}, query_id=99)).query_id == 99


def test_it_is_reproducible_from_its_seed():
    available = {"left": 9, "straight": 6, "right": 1}
    a = [RandomRouter(seed=3).route(query(available, query_id=i)).turn
         for i in range(20)]
    b = [RandomRouter(seed=3).route(query(available, query_id=i)).turn
         for i in range(20)]

    assert a == b


def test_it_actually_varies():
    """A router that always answers "straight" would pass every test above."""
    router = RandomRouter(seed=5)
    turns = {router.route(query({"left": 9, "straight": 6, "right": 1},
                                query_id=i)).turn for i in range(60)}

    assert turns == {"left", "straight", "right"}


def test_it_declines_a_dead_end_rather_than_inventing_a_turn():
    assert RandomRouter(seed=0).route(query({})) is None


def test_it_takes_the_only_way_out_when_there_is_one():
    for _ in range(20):
        decision = RandomRouter(seed=0).route(query({"right": 4}))
        assert decision.turn == "right" and decision.target_node_id == 4


def test_it_counts_what_it_decided():
    router = RandomRouter(seed=0)
    for index in range(5):
        router.route(query({"left": 9}, query_id=index))
    router.route(query({}))          # a dead end is not a decision

    assert router.decisions == 5


# ------------------------------------------------- routing over a real graph --

def test_every_decision_is_drivable_on_a_lattice():
    """End to end against the real topology: for every node and every approach
    heading, whatever the router picks must be a genuine neighbour."""
    from tools.track.graph import lattice

    graph = lattice(rows=4, columns=4, spacing=2.0)
    router = RandomRouter(seed=11)
    asked = 0

    for node_id in graph.nodes:
        for heading_deg in (0, 90, 180, 270):
            available = graph.turns_from(node_id, heading=math.radians(heading_deg))
            decision = router.route(query(available, node_id=node_id))
            if decision is None:
                assert not available
                continue
            asked += 1
            assert decision.target_node_id in graph.neighbours(node_id)

    assert asked > 0, "the lattice offered no turns at all"
