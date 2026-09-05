"""The transport enforces the schemas at the wire boundary.

The wire is typed protobuf now, so the envelope's own failure modes -- an
unknown schema name, a payload that does not match the name it was sent under --
cannot be constructed any more: the two directions are different types and the
stream says which is which. What protobuf does *not* enforce is what these tests
are mostly about.
"""

import pytest
from jsonschema import ValidationError

from temp_network_interface import network_pb2, validate_robot_to_network
from temp_network_interface.messages import (
    Battery,
    Cargo,
    Error,
    Fault,
    Mission,
    NetworkToRobot,
    Telemetry,
    Warning,
)
from temp_network_interface.transport import (
    decode_network_to_robot,
    decode_robot_to_network,
    encode_network_to_robot,
    encode_robot_to_network,
)

from .test_messages import status


def test_encode_produces_a_typed_message_not_an_envelope():
    wire = encode_robot_to_network(status())
    assert isinstance(wire, network_pb2.RobotToNetwork)
    assert wire.bot_id == 1


@pytest.mark.parametrize("mission", [
    Mission(type="PARK"),
    Mission(type="CHARGE"),
    Mission(type="HOLD"),
    Mission(type="IDLE"),
    Mission(type="CARGO", cargo=Cargo(
        cargo_id="6f1c2b7e-6d4a-4c1e-9b2f-0a5d3e8c7f11", state="EN_ROUTE")),
])
def test_every_mission_variant_survives_encode_decode(mission):
    """All five `oneof` cases, including the one that carries a payload.

    The four empty variants are the ones worth parametrising: setting a oneof
    case needs an instance even when the message has no fields, and forgetting
    that yields a mission that decodes as "none set".
    """
    original = status(mission=mission)
    assert decode_robot_to_network(encode_robot_to_network(original)) == original


def test_command_survives_encode_decode():
    original = NetworkToRobot(target_node_id=99, timestamp=5,
                              set_mission=Mission(type="CHARGE"))
    assert decode_network_to_robot(encode_network_to_robot(original)) == original


def test_fault_carries_a_warning_and_an_error_at_once():
    """`Fault` is deliberately not a oneof: the schema requires neither member
    and forbids neither, so both together must survive the wire."""
    original = status(fault=Fault(
        warning=Warning(type="OBSTACLE", current_node_id=190),
        error=Error(type="LIDAR_ERROR"),
    ))
    decoded = decode_robot_to_network(encode_robot_to_network(original))
    assert decoded == original
    assert decoded.fault.warning.current_node_id == 190
    assert decoded.fault.error.type == "LIDAR_ERROR"


def test_low_battery_warning_survives_encode_decode():
    original = status(fault=Fault(
        warning=Warning(type="LOW_BATTERY", percentage=12.5)))
    assert decode_robot_to_network(encode_robot_to_network(original)) == original


def test_enum_values_lose_their_protobuf_prefix_on_the_way_out():
    """Proto3 enum values share a scope, so the schema's bare `EN_ROUTE` has to
    be carried as `CARGO_STATE_EN_ROUTE`. The schema never sees the prefix."""
    original = status(mission=Mission(type="CARGO", cargo=Cargo(
        cargo_id="6f1c2b7e-6d4a-4c1e-9b2f-0a5d3e8c7f11", state="PICKUP")))
    wire = encode_robot_to_network(original)
    assert wire.mission.cargo.state == network_pb2.CARGO_STATE_PICKUP
    assert decode_robot_to_network(wire).mission.cargo.state == "PICKUP"


def test_zero_is_a_value_not_an_absence():
    """`Id` has `minimum: 0`, so bot 0 is a real bot. Required scalars are
    `optional` in the proto precisely so this round-trips rather than reading
    back as "nobody said"."""
    original = status(bot_id=0, region_id=0, latest_node_id=0, timestamp=0)
    wire = encode_robot_to_network(original)
    assert wire.HasField("bot_id")
    assert decode_robot_to_network(wire) == original


def test_missing_required_field_is_refused_at_decode():
    """What proto3 cannot say. An empty message is structurally valid protobuf;
    only the schema knows those fields were required, which is why validation
    stays at the boundary rather than being retired now the wire is typed."""
    with pytest.raises(ValidationError):
        decode_robot_to_network(network_pb2.RobotToNetwork())


def test_missing_field_fails_validation():
    document = status().to_dict()
    del document["telemetry"]
    with pytest.raises(ValidationError):
        validate_robot_to_network(document)


def test_out_of_range_percentage_is_refused_at_encode():
    """Another thing protobuf will not catch: a double is a double. The schema's
    `maximum: 100` is the only thing standing between a bad reading and the
    fleet's state."""
    with pytest.raises(ValidationError):
        encode_robot_to_network(status(telemetry=Telemetry(battery=Battery(
            percentage=140.0))))
