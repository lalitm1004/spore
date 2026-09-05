"""The transport enforces the schemas at the wire boundary."""

import json

import pytest
from jsonschema import ValidationError

from temp_network_interface import network_pb2, validate_robot_to_network
from temp_network_interface.messages import Mission, NetworkToRobot
from temp_network_interface.transport import decode, encode_network_to_robot, encode_robot_to_network

from .test_messages import status


def test_encode_robot_to_network_names_the_schema():
    envelope = encode_robot_to_network(status())
    assert envelope.schema == "robot-to-network"
    assert json.loads(envelope.json)["bot_id"] == 1


def test_robot_status_survives_encode_decode():
    original = status(mission=Mission(type="PARK"))
    assert decode(encode_robot_to_network(original)) == original


def test_command_survives_encode_decode():
    original = NetworkToRobot(target_node_id=99, timestamp=5, set_mission=Mission(type="CHARGE"))
    assert decode(encode_network_to_robot(original)) == original


def test_missing_field_fails_validation():
    document = status().to_dict()
    del document["telemetry"]
    with pytest.raises(ValidationError):
        validate_robot_to_network(document)


def test_unknown_schema_is_refused_at_decode():
    envelope = network_pb2.Message(schema="something-else", json="{}")
    with pytest.raises(ValueError):
        decode(envelope)


def test_off_contract_json_is_rejected_at_decode():
    envelope = network_pb2.Message(schema="robot-to-network", json="{}")
    with pytest.raises(ValidationError):
        decode(envelope)
