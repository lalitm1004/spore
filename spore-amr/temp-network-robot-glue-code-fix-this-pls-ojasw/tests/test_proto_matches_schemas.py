"""The proto and the schemas describe the same contract.

There are two definitions of this interface now -- `schemas/*.schema.json` and
`proto/network.proto` -- where the envelope had one. That is the price of a
typed wire, and it is only worth paying if something catches them drifting.
This is that something.

The schemas remain authoritative: these tests read them and demand the proto
keep up, never the other way round. Add a mission type or a fault field to a
schema and the failure lands here, naming what the proto is missing, rather than
surfacing as a message that silently loses a field in production.
"""

import json
import pathlib

import pytest

from temp_network_interface import network_pb2, schemas

# Schema `$defs` name -> protobuf message name, where they differ. The schema
# spells its variants `MissionPark`/`WarningObstacle` because JSON Schema has no
# sum type; the proto uses a `oneof`, so the variants are messages in their own
# right and carry the shorter name.
ALIASES = {
    "MissionPark": "Park",
    "MissionCharge": "Charge",
    "MissionHold": "Hold",
    "MissionIdle": "Idle",
    "MissionCargo": "Cargo",
    "WarningLowBattery": "LowBattery",
    "WarningObstacle": "Obstacle",
}

# `Id`, `Timestamp` and `CargoId` are scalar aliases in the schema; the proto
# spells them uint32/uint64/string inline rather than wrapping each in a
# message, so they have no counterpart to compare.
SCALAR_DEFS = {"Id", "Timestamp", "CargoId"}

ENUM_PREFIXES = {"CargoState": "CARGO_STATE_", "ErrorType": "ERROR_TYPE_"}


def load(name):
    return json.loads((schemas.SCHEMA_DIR / "{}.schema.json".format(name)).read_text())


def object_defs(document):
    """Every object definition in a schema, with its property names.

    A `type` property that is a `const` is dropped: it is the schema's
    discriminator for a sum type, and the proto encodes that as which `oneof`
    case is set rather than as a field. A `type` that is a `$ref` -- as on
    `Error` -- is real data and stays.
    """
    for name, definition in document["$defs"].items():
        if definition.get("type") != "object":
            continue
        properties = dict(definition.get("properties", {}))
        if "const" in properties.get("type", {}):
            properties.pop("type")
        yield name, set(properties)


SCHEMA_NAMES = [schemas.ROBOT_TO_NETWORK, schemas.NETWORK_TO_ROBOT]


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_every_schema_object_has_a_protobuf_message(schema_name):
    for name, _ in object_defs(load(schema_name)):
        expected = ALIASES.get(name, name)
        assert expected in network_pb2.DESCRIPTOR.message_types_by_name, (
            "schema defines {!r} but the proto has no {!r} message".format(
                name, expected))


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_message_fields_match_schema_properties(schema_name):
    for name, properties in object_defs(load(schema_name)):
        expected = ALIASES.get(name, name)
        message = network_pb2.DESCRIPTOR.message_types_by_name[expected]
        fields = {field.name for field in message.fields}
        if name == "MissionCargo":
            # The schema nests the cargo one level deeper than the proto does:
            # `{type: CARGO, cargo: {...}}` against the `Mission.cargo` case,
            # whose message *is* the cargo. Compare against what it holds.
            continue
        assert fields == properties, (
            "{} ({}): proto has {}, schema has {}".format(
                name, expected, sorted(fields), sorted(properties)))


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_enum_values_match(schema_name):
    document = load(schema_name)
    for name, definition in document["$defs"].items():
        if "enum" not in definition:
            continue
        prefix = ENUM_PREFIXES[name]
        values = {v.name for v in
                  network_pb2.DESCRIPTOR.enum_types_by_name[name].values}
        # Proto3 requires a zero value; it doubles as the "absent or
        # unrecognised" sentinel that `required` would otherwise have caught.
        expected = {prefix + v for v in definition["enum"]} | {prefix + "UNSPECIFIED"}
        assert values == expected, "{}: proto has {}, schema has {}".format(
            name, sorted(values), sorted(expected))


@pytest.mark.parametrize("schema_name", SCHEMA_NAMES)
def test_scalar_aliases_are_not_expected_as_messages(schema_name):
    """Guards the exemption above: if `Id` ever grows into an object, this stops
    silently excusing it from the field comparison."""
    document = load(schema_name)
    for name in SCALAR_DEFS:
        if name in document["$defs"]:
            assert document["$defs"][name].get("type") != "object", (
                "{} became an object; it now needs a protobuf message".format(name))


def test_mission_oneof_covers_every_schema_variant():
    """The `oneof` cases and the schema's `Mission.oneOf` branches are the same
    set. A sixth mission type added to the schema fails here."""
    document = load(schemas.ROBOT_TO_NETWORK)
    branches = {ref["$ref"].rsplit("/", 1)[-1]
                for ref in document["$defs"]["Mission"]["oneOf"]}
    expected = {ALIASES[b].lower() for b in branches}
    cases = {f.name for f in
             network_pb2.DESCRIPTOR.message_types_by_name["Mission"]
             .oneofs_by_name["kind"].fields}
    assert cases == expected


def test_warning_oneof_covers_every_schema_variant():
    document = load(schemas.ROBOT_TO_NETWORK)
    branches = {ref["$ref"].rsplit("/", 1)[-1]
                for ref in document["$defs"]["Warning"]["oneOf"]}
    # LowBattery -> low_battery, Obstacle -> obstacle
    expected = {"low_battery" if ALIASES[b] == "LowBattery" else ALIASES[b].lower()
                for b in branches}
    cases = {f.name for f in
             network_pb2.DESCRIPTOR.message_types_by_name["Warning"]
             .oneofs_by_name["kind"].fields}
    assert cases == expected


def test_the_wire_still_cannot_carry_a_turn():
    """The architectural invariant, enforced rather than documented.

    `NetworkToRobot` names a node and never a direction: the network layer
    routes, and the robot derives the bearing from the map it also holds. A
    `turn`, a `bearing`, or a menu of legal exits appearing on this message
    would be a different system.
    """
    fields = {f.name for f in
              network_pb2.DESCRIPTOR.message_types_by_name["NetworkToRobot"].fields}
    assert fields == {"target_node_id", "set_mission", "timestamp"}


def test_proto_file_is_the_one_the_stubs_were_generated_from():
    """A stale `network_pb2.py` is invisible until something behaves oddly at
    runtime. Compare the message set against the proto source itself."""
    proto = (pathlib.Path(__file__).resolve().parent.parent
             / "proto" / "network.proto").read_text()
    declared = set(__import__("re").findall(r"^message\s+(\w+)", proto, __import__("re").M))
    generated = set(network_pb2.DESCRIPTOR.message_types_by_name)
    assert declared == generated, (
        "proto declares {}, stubs have {} -- run tools/gen_proto.py".format(
            sorted(declared - generated), sorted(generated - declared)))
