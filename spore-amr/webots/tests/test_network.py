"""The stand-in network layer: its wire format and its routing.

Both halves matter. The wire format is the boundary the real TypeScript layer
will inherit, so it is tested as a contract; the routing is a mock, so it is
tested only for the properties a robot depends on -- legality, reproducibility,
and answering the question it was actually asked.
"""

import json
import math

import pytest

from robot.network import PROCEED, WAIT, Decision, Query


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


def test_a_wait_carries_a_hold_and_names_no_lane():
    """The kind that did not exist before: a robot told to stay put.

    Without it, "wait" could only be expressed by not answering -- and a robot
    that hears nothing sits there for the rest of its shift, because it only
    asks again on reaching the next node.
    """
    decision = Decision(query_id=4, turn="", target_node_id=0,
                        kind=WAIT, hold_ms=1500, because="giving way to [9]")
    parsed = Decision.from_json(decision.to_json())
    assert parsed.is_wait
    assert parsed.hold_ms == 1500
    assert parsed.because == "giving way to [9]"


def test_an_answer_without_a_kind_is_taken_as_proceed():
    """Additive on purpose: a network layer that only sends a turn still works."""
    parsed = Decision.from_json(
        '{"query_id":3,"turn":"right","target_node_id":7}')
    assert parsed.kind == PROCEED
    assert parsed.turn == "right"


def test_an_unknown_kind_is_rejected():
    import pytest

    with pytest.raises(ValueError):
        Decision.from_json(
            '{"query_id":5,"kind":"NONSENSE","turn":"left","target_node_id":7}')
