"""The proto and the shared schemas describe the same contract.

There are two definitions of the robot link now — `shared/schemas/*.schema.json`
and `proto/robot.proto` — where there was one. That is the price of a typed
wire, and it is only worth paying if something catches them drifting. This is
that something.

The schemas stay authoritative: these tests read them and demand the proto keep
up, never the other way round. Add a mission type or a fault field to a schema
and the failure lands here, naming what the proto is missing, rather than
surfacing later as a message that silently drops a field.

Adapted from the version on `webots-implementation`, which wrote it for the
same proto under a central service. What is new is `EXTENSIONS`: this fleet
answers a robot *at a junction* as well as telling it where to go, so four
fields exist here that are in neither schema. Enumerating them is what keeps
"one wire" a fact rather than an intention — an addition to either side has to
be declared here or it fails.
"""
from __future__ import annotations

import json
import pathlib
import re

import pytest

from proto import robot_pb2

SCHEMA_DIR = pathlib.Path(__file__).resolve().parents[2] / "shared" / "schemas"
ROBOT_TO_NETWORK = "robot-to-network"
NETWORK_TO_ROBOT = "network-to-robot"
SCHEMA_NAMES = [ROBOT_TO_NETWORK, NETWORK_TO_ROBOT]

#: Schema `$defs` name -> protobuf message name, where they differ. The schema
#: spells its variants `MissionPark`/`WarningObstacle` because JSON Schema has
#: no sum type; the proto uses a `oneof`, so the variants are messages in their
#: own right and carry the shorter name.
ALIASES = {
    "MissionPark": "Park",
    "MissionCharge": "Charge",
    "MissionHold": "Hold",
    "MissionIdle": "Idle",
    "MissionCargo": "Cargo",
    "WarningLowBattery": "LowBattery",
    "WarningObstacle": "Obstacle",
}

#: `Id`, `Timestamp` and `CargoId` are scalar aliases in the schema; the proto
#: spells them uint32/uint64/string inline rather than wrapping each in a
#: message, so they have no counterpart to compare.
SCALAR_DEFS = {"Id", "Timestamp", "CargoId"}

ENUM_PREFIXES = {"CargoState": "CARGO_STATE_", "ErrorType": "ERROR_TYPE_"}

#: Fields this fleet adds, and the whole of them.
#:
#: The schemas describe a robot *reporting* and a network *commanding*. They do
#: not describe a robot stopped at a junction asking which way to go, which is
#: the conversation this fleet actually has, and a destination alone cannot say
#: "hold 800 ms and ask again" or "stand aside at node 412". Silence is the one
#: answer a blind robot cannot recover from, so being able to say wait is not
#: decoration (PROTOCOL.md §16.2).
#:
#: Every one of these is a deliberate divergence. Adding a fifth without adding
#: it here fails, which is the point: the next person to extend one side has to
#: say so out loud.
EXTENSIONS = {
    "RobotToNetwork": {"available", "heading_rad", "query_id"},
    "NetworkToRobot": {"kind", "hold_ms", "because", "query_id"},
}


def load(name: str) -> dict:
    return json.loads((SCHEMA_DIR / f"{name}.schema.json").read_text())


def object_defs(document: dict):
    """Every object definition in a schema, with its property names.

    A `type` property that is a `const` is dropped: it is the schema's
    discriminator for a sum type, and the proto encodes that as which `oneof`
    case is set rather than as a field. A `type` that is a `$ref` — as on
    `Error` — is real data and stays.
    """
    for name, definition in document["$defs"].items():
        if definition.get("type") != "object":
            continue
        properties = dict(definition.get("properties", {}))
        if "const" in properties.get("type", {}):
            properties.pop("type")
        yield name, set(properties)


def test_the_schemas_are_where_we_think_they_are():
    """A path that has silently moved once already, and would make every test
    below pass by finding nothing."""
    assert SCHEMA_DIR.is_dir(), f"no schema directory at {SCHEMA_DIR}"
    for name in SCHEMA_NAMES:
        assert (SCHEMA_DIR / f"{name}.schema.json").is_file(), name


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_every_schema_object_has_a_protobuf_message(schema_name):
    for name, _ in object_defs(load(schema_name)):
        expected = ALIASES.get(name, name)
        assert expected in robot_pb2.DESCRIPTOR.message_types_by_name, (
            f"schema defines {name!r} but the proto has no {expected!r} message")


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_the_proto_carries_every_property_the_schema_declares(schema_name):
    """Nothing in a schema may go missing from the wire."""
    for name, properties in object_defs(load(schema_name)):
        expected = ALIASES.get(name, name)
        if name == "MissionCargo":
            # The schema nests the cargo one level deeper than the proto does:
            # `{type: CARGO, cargo: {...}}` against the `Mission.cargo` case,
            # whose message *is* the cargo. Compare against what it holds.
            continue
        message = robot_pb2.DESCRIPTOR.message_types_by_name[expected]
        fields = {field.name for field in message.fields}
        missing = properties - fields
        assert not missing, f"{name} ({expected}): proto is missing {sorted(missing)}"


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_every_extra_field_is_a_declared_extension(schema_name):
    """The other direction, and the one that keeps the two honest.

    A field on the wire that no schema mentions is either a deliberate
    extension — in which case it is listed in `EXTENSIONS` with a reason in the
    proto — or it is drift.
    """
    for name, properties in object_defs(load(schema_name)):
        expected = ALIASES.get(name, name)
        if name == "MissionCargo":
            continue
        message = robot_pb2.DESCRIPTOR.message_types_by_name[expected]
        fields = {field.name for field in message.fields}
        extra = fields - properties
        allowed = EXTENSIONS.get(expected, set())
        assert extra == allowed, (
            f"{name} ({expected}): undeclared extra fields "
            f"{sorted(extra - allowed)}, declared but absent {sorted(allowed - extra)}")


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_enum_values_match(schema_name):
    document = load(schema_name)
    for name, definition in document["$defs"].items():
        if "enum" not in definition:
            continue
        prefix = ENUM_PREFIXES[name]
        values = {v.name for v in robot_pb2.DESCRIPTOR.enum_types_by_name[name].values}
        # Proto3 requires a zero value; it doubles as the "absent or
        # unrecognised" sentinel that `required` would otherwise have caught.
        expected = {prefix + v for v in definition["enum"]} | {prefix + "UNSPECIFIED"}
        assert values == expected, (
            f"{name}: proto has {sorted(values)}, schema has {sorted(expected)}")


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_scalar_aliases_are_not_expected_as_messages(schema_name):
    """Guards the exemption above: if `Id` ever grows into an object, this stops
    silently excusing it from the field comparison."""
    document = load(schema_name)
    for name in SCALAR_DEFS:
        if name in document["$defs"]:
            assert document["$defs"][name].get("type") != "object", (
                f"{name} became an object; it now needs a protobuf message")


def test_mission_oneof_covers_every_schema_variant():
    """The `oneof` cases and the schema's `Mission.oneOf` branches are the same
    set. A sixth mission type added to the schema fails here."""
    document = load(ROBOT_TO_NETWORK)
    branches = {ref["$ref"].rsplit("/", 1)[-1]
                for ref in document["$defs"]["Mission"]["oneOf"]}
    expected = {ALIASES[b].lower() for b in branches}
    cases = {f.name for f in
             robot_pb2.DESCRIPTOR.message_types_by_name["Mission"]
             .oneofs_by_name["kind"].fields}
    assert cases == expected


def test_warning_oneof_covers_every_schema_variant():
    document = load(ROBOT_TO_NETWORK)
    branches = {ref["$ref"].rsplit("/", 1)[-1]
                for ref in document["$defs"]["Warning"]["oneOf"]}
    expected = {"low_battery" if ALIASES[b] == "LowBattery" else ALIASES[b].lower()
                for b in branches}
    cases = {f.name for f in
             robot_pb2.DESCRIPTOR.message_types_by_name["Warning"]
             .oneofs_by_name["kind"].fields}
    assert cases == expected


def test_the_wire_still_cannot_carry_a_turn():
    """The architectural invariant, enforced rather than documented.

    `NetworkToRobot` names a node and never a direction: the network layer
    routes, and the robot derives the bearing from the map it also holds. That
    is exact, because lanes are straight — and the firmware bears it out, having
    never read anything but `bearing` and `heading` off a TURN command.

    A `turn` or a `bearing` appearing here would be a different system. The
    legal exits go the *other* way, on `RobotToNetwork.available`, because the
    robot is the one that knows what is physically possible from where it
    stands.
    """
    fields = {f.name for f in
              robot_pb2.DESCRIPTOR.message_types_by_name["NetworkToRobot"].fields}
    assert not fields & {"turn", "bearing", "direction", "heading_rad", "available"}


def test_the_obstruction_node_survives_the_wire():
    """The field that makes a reported blockage real rather than injected.

    `RobotState.fault` was a flat string, so a robot's OBSTACLE warning lost the
    node it was about and nothing could build an obstruction from it. The schema
    always carried `current_node_id`; the wire now does too.
    """
    message = robot_pb2.RobotToNetwork(
        fault=robot_pb2.Fault(warning=robot_pb2.Warning(
            obstacle=robot_pb2.Obstacle(current_node_id=455))))
    restored = robot_pb2.RobotToNetwork.FromString(message.SerializeToString())
    assert restored.fault.warning.obstacle.current_node_id == 455


def test_asking_and_reporting_are_told_apart_by_available():
    """What makes this one wire rather than two bolted together.

    A report with legal exits is a robot stopped at a junction waiting to be
    told where to go. A report without them is telemetry. Both update position;
    only the first is answered.
    """
    telemetry = robot_pb2.RobotToNetwork(latest_node_id=5)
    question = robot_pb2.RobotToNetwork(latest_node_id=5, available=[6, 7])
    assert not telemetry.available
    assert list(question.available) == [6, 7]


def test_proto_file_is_the_one_the_stubs_were_generated_from():
    """A stale `robot_pb2.py` is invisible until something behaves oddly at
    runtime. Compare the message set against the proto source itself."""
    proto = (pathlib.Path(__file__).resolve().parents[1]
             / "proto" / "robot.proto").read_text()
    declared = set(re.findall(r"^message\s+(\w+)", proto, re.M))
    generated = set(robot_pb2.DESCRIPTOR.message_types_by_name)
    assert declared == generated, (
        f"proto declares {sorted(declared - generated)}, "
        f"stubs have {sorted(generated - declared)} -- regenerate them "
        "(see README.md)")


# ---- The spec embeds the wire, and the wire is the file --------------------------

PROTOCOL = pathlib.Path(__file__).resolve().parents[1] / "PROTOCOL.md"
EMBEDDED = [
    ("## 11. Proto File", "proto/fleet.proto"),
    ("### 11.1 The robot link", "proto/robot.proto"),
    ("### 11.2 Orders in", "proto/controlplane.proto"),
]


def _fenced_after(text: str, heading: str) -> str:
    start = text.index(heading)
    a = text.index("```protobuf\n", start) + len("```protobuf\n")
    return text[a:text.index("\n```", a)].strip()


@pytest.mark.parametrize(("heading", "proto"), EMBEDDED)
def test_protocol_md_embeds_the_proto_verbatim(heading, proto):
    """§11 calls itself "Proto File -- Complete". Complete means the file, not a
    hand-maintained copy that was complete when it was written: the two had
    already drifted once (the doc still had an RPC the file had lost). Now the
    doc block is regenerated from the file, and this fails if anyone edits one
    without the other."""
    want = (PROTOCOL.parent / proto).read_text().strip()
    assert _fenced_after(PROTOCOL.read_text(), heading) == want, (
        f"{proto} and the block under {heading!r} in PROTOCOL.md differ; "
        "paste the file into the doc (or run the re-embed in the commit that changed it)")


# ---- The hand-written mirrors ----------------------------------------------------
# The planner's `Query`/`Decision` and the robot side's copies are dataclasses,
# not protos, on purpose: neither half should reason in wire types. That is
# only safe if the mirrors stay a subset of the wire, plus fields we *say* are
# local. This pins which.

WEBOTS = pathlib.Path(__file__).resolve().parents[2] / "webots"

#: Fields a mirror carries that the wire does not, each with its reason.
LOCAL_ONLY = {
    "Query": {"node_type"},          # the robot reads it off the tile; the fleet has the map
    "Decision": set(),
    "RobotState": {"state"},         # the FSM state is not on the wire, by design
}

#: Fields the robot's own mirror carries that the planner's does not, because
#: the two are not symmetric. `NetworkToRobot.set_mission` is on the wire and
#: the robot has to read it -- it is how a robot learns it has a job at all --
#: but the planner never produces it: `reply_of` fills it in from the bot's own
#: job, beside the planner's answer rather than inside it. So these are wire
#: fields the robot decodes and the planner has no business knowing about.
ROBOT_ONLY_DECISION = {"mission", "cargo_id", "cargo_state"}


def _dataclass_fields(cls) -> set[str]:
    return set(cls.__dataclass_fields__)


def test_the_planners_query_and_decision_are_subsets_of_the_wire():
    from planning.decide import Decision, Query
    up = {f.name for f in robot_pb2.RobotToNetwork.DESCRIPTOR.fields}
    down = {f.name for f in robot_pb2.NetworkToRobot.DESCRIPTOR.fields}
    assert _dataclass_fields(Query) - LOCAL_ONLY["Query"] <= up | {"node_id"}, \
        "Query carries a field the wire cannot deliver"
    assert _dataclass_fields(Decision) - LOCAL_ONLY["Decision"] <= down, \
        "Decision carries a field the wire cannot deliver"


def test_the_robot_sides_query_and_decision_match_the_planners():
    """Two mirrors of one wire on two sides of it. They may differ from the
    proto only in the ways listed above, and never from each other."""
    import sys
    sys.path.insert(0, str(WEBOTS))
    from robot.network import Decision as RDecision, Query as RQuery
    from planning.decide import Decision, Query
    assert _dataclass_fields(RQuery) == _dataclass_fields(Query)
    assert _dataclass_fields(RDecision) - ROBOT_ONLY_DECISION == _dataclass_fields(Decision)
    assert ROBOT_ONLY_DECISION <= _dataclass_fields(RDecision), \
        "the robot stopped reading a mission field the wire still carries"


def test_robot_state_is_the_wire_flattened_plus_what_we_say_is_local():
    from bot import RobotState
    # What `robot_service.state_of` can fill from a RobotToNetwork.
    reachable = {"latest_node_id", "region_id", "battery", "mission", "fault", "job_id", "cargo_state"}
    assert _dataclass_fields(RobotState) == reachable | LOCAL_ONLY["RobotState"], \
        "RobotState gained a field nothing on the wire can fill -- add it to LOCAL_ONLY with a reason, or to the proto"
