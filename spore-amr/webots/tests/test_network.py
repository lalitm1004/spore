"""The stand-in network layer: its wire format and its routing.

Both halves matter. The wire format is the boundary the real TypeScript layer
will inherit, so it is tested against `shared/schemas/` as a contract; the
routing is a mock, so it is tested only for the properties a robot depends on
-- legality, reproducibility, and answering the question it was actually asked.

The contract is deliberately narrow. A robot reports **where it is**; it is
told **which node to go to next**. It does not offer a menu of turns and it is
not sent a "left"/"right", because `additionalProperties` is false on both
schemas and neither field is in either of them. Left and right are the robot's
own business: it holds the map, so it can work out the bearing to a named
neighbour without being told which way that is.
"""

import json
import pathlib

import pytest

from robot.network import Decision, Query, RandomRouter
from tools.track.graph import lattice

SCHEMAS = pathlib.Path(__file__).resolve().parents[2] / "shared" / "schemas"


def query(node_id=5, bot_id=1, region_id=2):
    return Query(bot_id=bot_id, region_id=region_id, latest_node_id=node_id,
                 battery_percent=87.5, timestamp=1700000000)


def required(schema_name, definition):
    document = json.loads((SCHEMAS / schema_name).read_text())
    return document["$defs"][definition]


# ------------------------------------------------------------ the contract --

def test_a_query_carries_exactly_what_the_shared_schema_allows():
    """`additionalProperties: false`, so an extra field is not a harmless
    extension -- it is a message the real network layer must reject."""
    schema = required("robot-to-network.schema.json", "RobotToNetwork")
    payload = json.loads(query().to_json())

    assert set(schema["required"]) <= set(payload)
    assert set(payload) <= set(schema["properties"])


def test_a_query_does_not_offer_a_menu_of_turns():
    """The robot says where it is. Choosing where it goes is the network
    layer's job, and it holds its own map to do it."""
    payload = json.loads(query().to_json())

    assert "available" not in payload
    assert payload["latest_node_id"] == 5


def test_a_decision_carries_exactly_what_the_shared_schema_allows():
    schema = required("network-to-robot.schema.json", "NetworkToRobot")
    payload = json.loads(Decision(target_node_id=9, timestamp=17).to_json())

    assert set(schema["required"]) <= set(payload)
    assert set(payload) <= set(schema["properties"])


def test_a_decision_names_a_node_and_never_a_direction():
    """A `turn` field would be rejected by the schema, and the robot does not
    need one: it can see from the map which way node 9 is."""
    payload = json.loads(Decision(target_node_id=9).to_json())

    assert "turn" not in payload
    assert payload["target_node_id"] == 9


def test_query_round_trips_over_the_wire():
    restored = Query.from_json(query(node_id=42, bot_id=7).to_json())

    assert restored.latest_node_id == 42
    assert restored.bot_id == 7
    assert restored.battery_percent == pytest.approx(87.5)


def test_decision_round_trips_over_the_wire():
    restored = Decision.from_json(Decision(target_node_id=9, timestamp=3).to_json())

    assert (restored.target_node_id, restored.timestamp) == (9, 3)


def test_wire_format_is_json_lines():
    """The transport splits on newlines, so a payload must not contain one."""
    for text in (query().to_json(), Decision(target_node_id=9).to_json()):
        assert "\n" not in text
        json.loads(text)


def test_a_decision_without_a_target_is_rejected():
    with pytest.raises((ValueError, KeyError)):
        Decision.from_json('{"timestamp": 1}')


# -------------------------------------------------------------- the router --

def graph():
    return lattice(rows=4, columns=4, spacing=2.0)


def test_it_only_ever_names_a_neighbour_of_where_the_robot_is():
    """The property the whole design rests on: whatever comes back, the robot
    can drive to it. A blind pick from the whole map would name walls."""
    router = RandomRouter(graph(), seed=7)

    for node_id in graph().nodes:
        for _ in range(10):
            decision = router.route(query(node_id=node_id))
            assert decision.target_node_id in graph().neighbours(node_id)


def test_it_is_reproducible_from_its_seed():
    a = [RandomRouter(graph(), seed=3).route(query(node_id=5)).target_node_id
         for _ in range(20)]
    b = [RandomRouter(graph(), seed=3).route(query(node_id=5)).target_node_id
         for _ in range(20)]

    assert a == b


def test_it_actually_varies():
    """A router that always returned the same neighbour would pass the tests
    above."""
    router = RandomRouter(graph(), seed=5)
    targets = {router.route(query(node_id=5)).target_node_id for _ in range(60)}

    assert targets == set(graph().neighbours(5))


def test_it_declines_a_node_it_has_never_heard_of():
    """Rather than inventing somewhere to go."""
    assert RandomRouter(graph(), seed=0).route(query(node_id=9999)) is None


def test_it_takes_the_only_way_out_of_a_dead_end():
    """A charging bay is a degree-1 spur, so its one neighbour is the way back
    to the corridor. This is the case that used to strand robots: nothing was
    offered, and a robot with nowhere to go sat in the bay for the whole run."""
    nodes = list(graph().nodes.values())
    from tools.track.graph import Edge, Graph, Node
    bay = Node(node_id=99, x=0.0, y=-9.0, kind="CH")
    spur = Graph(nodes + [bay], list(graph().edges) + [Edge(5, 99)])
    router = RandomRouter(spur, seed=0)

    for _ in range(20):
        assert router.route(query(node_id=99)).target_node_id == 5


def test_it_counts_what_it_decided():
    router = RandomRouter(graph(), seed=0)
    for _ in range(5):
        router.route(query(node_id=5))
    router.route(query(node_id=9999))      # unknown node is not a decision

    assert router.decisions == 5


def test_it_echoes_the_timestamp_it_was_asked_with():
    """The only correlation token the schema has. A reply carrying a different
    one is an answer to an earlier junction."""
    decision = RandomRouter(graph(), seed=1).route(query(node_id=5))

    assert decision.timestamp == 1700000000


# ------------------------------------------------------- not going backwards --

def test_it_does_not_send_a_robot_straight_back_where_it_came_from():
    """`turns_from` used to exclude the arrival lane, which is what kept the
    random walk moving. That exclusion is the router's business now, and it
    needs no new field to do it: it answered the last question, so it knows
    where this robot was before it was here."""
    router = RandomRouter(graph(), seed=4)

    first = router.route(query(bot_id=1, node_id=5))
    for _ in range(30):
        nxt = router.route(query(bot_id=1, node_id=first.target_node_id))
        assert nxt.target_node_id != 5


def test_each_robot_is_tracked_separately():
    """One router per robot in this system, but the field is on the wire, so
    two bots on one router must not inherit each other's history."""
    router = RandomRouter(graph(), seed=4)
    router.route(query(bot_id=1, node_id=5))       # bot 1 leaves node 5

    targets = {router.route(query(bot_id=2, node_id=6)).target_node_id
               for _ in range(40)}

    assert targets == set(graph().neighbours(6))


def test_it_still_reverses_out_of_a_dead_end():
    """Backtracking is avoided, not forbidden. A charging bay has one
    neighbour and it is the way the robot came in; refusing to answer would
    strand it exactly as before."""
    from tools.track.graph import Edge, Graph, Node
    bay = Node(node_id=99, x=0.0, y=-9.0, kind="CH")
    spur = Graph(list(graph().nodes.values()) + [bay],
                 list(graph().edges) + [Edge(5, 99)])
    router = RandomRouter(spur, seed=0)

    router.route(query(bot_id=1, node_id=5))       # may or may not send it in
    for _ in range(20):
        assert router.route(query(bot_id=1, node_id=99)).target_node_id == 5
