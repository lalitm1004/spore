"""Marshalling between typed domain messages and the gRPC envelope.

This is the only module (besides `client.py` and `server.py`) that imports the
generated protobuf, so `messages.py` and `schemas.py` stay pure and importable
without grpc.

Validation happens here, at the boundary where a payload crosses the wire: the
shared schemas are authoritative, so a document that fails validation never
leaves the process and a payload received with a schema name we do not know is
refused rather than guessed at.
"""

import json

from temp_network_interface import network_pb2, schemas
from temp_network_interface.messages import NetworkToRobot, RobotToNetwork


def encode_robot_to_network(message: RobotToNetwork) -> network_pb2.Message:
    document = message.to_dict()
    schemas.validate_robot_to_network(document)
    return network_pb2.Message(
        schema=schemas.ROBOT_TO_NETWORK,
        json=json.dumps(document, separators=(",", ":")),
    )


def encode_network_to_robot(message: NetworkToRobot) -> network_pb2.Message:
    document = message.to_dict()
    schemas.validate_network_to_robot(document)
    return network_pb2.Message(
        schema=schemas.NETWORK_TO_ROBOT,
        json=json.dumps(document, separators=(",", ":")),
    )


def decode(envelope: network_pb2.Message):
    """Turn a wire envelope back into a typed message, validating the payload."""
    document = json.loads(envelope.json)
    if envelope.schema == schemas.ROBOT_TO_NETWORK:
        schemas.validate_robot_to_network(document)
        return RobotToNetwork.from_dict(document)
    if envelope.schema == schemas.NETWORK_TO_ROBOT:
        schemas.validate_network_to_robot(document)
        return NetworkToRobot.from_dict(document)
    raise ValueError("unknown schema {!r}".format(envelope.schema))
