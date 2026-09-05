"""Marshalling between typed domain messages and the typed protobuf wire.

This is the only module (besides `client.py` and `server.py`) that imports the
generated protobuf, so `messages.py` and `schemas.py` stay pure and importable
without grpc.

**The schema document is the pivot.** Encoding goes domain -> document ->
validate -> protobuf, and decoding goes protobuf -> document -> validate ->
domain. Nothing converts a domain object to a protobuf directly, which buys
three things: validation stays at the wire boundary where it always was, the
document form is the schemas' own so `messages.py` needs no protobuf awareness,
and a required field missing from the wire shows up as a schema error naming the
field rather than as a zero.

That last point is the reason validation is kept rather than retired now that
the wire is typed. Protobuf enforces *shape* -- unknown fields cannot be
constructed, and a mission is exactly one of five. It does not enforce
`required`, because proto3 has no such concept and every Id has `minimum: 0`, so
an absent field and a present zero are the same bytes. Nor does it enforce
`minimum`/`maximum` on a percentage, or the UUID pattern on a cargo id. Those
are the schemas' to keep, and they are load-bearing here, not redundant.

The two directions no longer share a message type, so there is no envelope and
no schema name to dispatch on: the stream a message arrived on already says
which of the two it is, and the compiler knows it.
"""

from temp_network_interface import network_pb2, schemas
from temp_network_interface.messages import NetworkToRobot, RobotToNetwork

# Schema mission `type` const <-> the protobuf oneof case that stands for it.
# Four carry no payload; CARGO is the one with a body, handled separately.
_MISSION_CASE = {"PARK": "park", "CHARGE": "charge", "HOLD": "hold", "IDLE": "idle"}
_MISSION_TYPE = {case: name for name, case in _MISSION_CASE.items()}

# The payload-free variants, as message classes. Setting a oneof case needs an
# instance even when the message is empty -- that is what marks the case as set.
_MISSION_EMPTY = {
    "PARK": network_pb2.Park,
    "CHARGE": network_pb2.Charge,
    "HOLD": network_pb2.Hold,
    "IDLE": network_pb2.Idle,
}

# Schema warning `type` const <-> protobuf oneof case.
_WARNING_CASE = {"LOW_BATTERY": "low_battery", "OBSTACLE": "obstacle"}
_WARNING_TYPE = {case: name for name, case in _WARNING_CASE.items()}

# Protobuf enums are prefixed -- proto3 enum values share a C++ scope, so two
# enums in one file cannot both have a bare `PICKUP`. The schema's values are
# the unprefixed halves.
_CARGO_STATE_PREFIX = "CARGO_STATE_"
_ERROR_TYPE_PREFIX = "ERROR_TYPE_"


# --------------------------------------------------------------- document -> pb


def _mission_to_pb(doc: dict) -> network_pb2.Mission:
    kind = doc["type"]
    if kind == "CARGO":
        cargo = doc["cargo"]
        return network_pb2.Mission(cargo=network_pb2.Cargo(
            cargo_id=cargo["cargo_id"],
            state=network_pb2.CargoState.Value(_CARGO_STATE_PREFIX + cargo["state"]),
        ))
    return network_pb2.Mission(**{_MISSION_CASE[kind]: _MISSION_EMPTY[kind]()})


def _fault_to_pb(doc: dict) -> network_pb2.Fault:
    # Not a oneof: the schema requires neither member and forbids neither, so a
    # fault may carry a warning, an error, both, or nothing at all.
    fault = network_pb2.Fault()
    if "warning" in doc:
        warning = doc["warning"]
        if warning["type"] == "LOW_BATTERY":
            fault.warning.low_battery.percentage = warning["percentage"]
        else:
            fault.warning.obstacle.current_node_id = warning["current_node_id"]
    if "error" in doc:
        fault.error.type = network_pb2.ErrorType.Value(
            _ERROR_TYPE_PREFIX + doc["error"]["type"])
    return fault


def _robot_to_network_to_pb(doc: dict) -> network_pb2.RobotToNetwork:
    message = network_pb2.RobotToNetwork(
        bot_id=doc["bot_id"],
        region_id=doc["region_id"],
        latest_node_id=doc["latest_node_id"],
        timestamp=doc["timestamp"],
        mission=_mission_to_pb(doc["mission"]),
        telemetry=network_pb2.Telemetry(battery=network_pb2.Battery(
            percentage=doc["telemetry"]["battery"]["percentage"])),
    )
    if "fault" in doc:
        message.fault.CopyFrom(_fault_to_pb(doc["fault"]))
    return message


def _network_to_robot_to_pb(doc: dict) -> network_pb2.NetworkToRobot:
    message = network_pb2.NetworkToRobot(
        target_node_id=doc["target_node_id"],
        timestamp=doc["timestamp"],
    )
    if "set_mission" in doc:
        message.set_mission.CopyFrom(_mission_to_pb(doc["set_mission"]))
    return message


# --------------------------------------------------------------- pb -> document
#
# Every read is presence-checked. A field the sender never set is left out of
# the document rather than defaulted, so a missing *required* field fails schema
# validation naming that field instead of arriving silently as a zero.


def _mission_to_doc(message: network_pb2.Mission) -> dict:
    case = message.WhichOneof("kind")
    if case is None:
        return {}                       # no mission set; the schema will object
    if case == "cargo":
        cargo = message.cargo
        doc = {"type": "CARGO", "cargo": {}}
        if cargo.HasField("cargo_id"):
            doc["cargo"]["cargo_id"] = cargo.cargo_id
        if cargo.HasField("state"):
            doc["cargo"]["state"] = network_pb2.CargoState.Name(
                cargo.state)[len(_CARGO_STATE_PREFIX):]
        return doc
    return {"type": _MISSION_TYPE[case]}


def _fault_to_doc(fault: network_pb2.Fault) -> dict:
    doc = {}
    if fault.HasField("warning"):
        case = fault.warning.WhichOneof("kind")
        if case == "low_battery":
            doc["warning"] = {"type": "LOW_BATTERY"}
            if fault.warning.low_battery.HasField("percentage"):
                doc["warning"]["percentage"] = fault.warning.low_battery.percentage
        elif case == "obstacle":
            doc["warning"] = {"type": "OBSTACLE"}
            if fault.warning.obstacle.HasField("current_node_id"):
                doc["warning"]["current_node_id"] = \
                    fault.warning.obstacle.current_node_id
    if fault.HasField("error") and fault.error.HasField("type"):
        doc["error"] = {"type": network_pb2.ErrorType.Name(
            fault.error.type)[len(_ERROR_TYPE_PREFIX):]}
    return doc


def _robot_to_network_to_doc(message: network_pb2.RobotToNetwork) -> dict:
    doc = {}
    for field in ("bot_id", "region_id", "latest_node_id", "timestamp"):
        if message.HasField(field):
            doc[field] = getattr(message, field)
    if message.HasField("mission"):
        doc["mission"] = _mission_to_doc(message.mission)
    if message.HasField("telemetry"):
        battery = message.telemetry.battery
        doc["telemetry"] = {"battery": {}}
        if message.telemetry.HasField("battery") and battery.HasField("percentage"):
            doc["telemetry"]["battery"]["percentage"] = battery.percentage
    if message.HasField("fault"):
        doc["fault"] = _fault_to_doc(message.fault)
    return doc


def _network_to_robot_to_doc(message: network_pb2.NetworkToRobot) -> dict:
    doc = {}
    for field in ("target_node_id", "timestamp"):
        if message.HasField(field):
            doc[field] = getattr(message, field)
    if message.HasField("set_mission"):
        doc["set_mission"] = _mission_to_doc(message.set_mission)
    return doc


# ------------------------------------------------------------------ public API


def encode_robot_to_network(message: RobotToNetwork) -> network_pb2.RobotToNetwork:
    document = message.to_dict()
    schemas.validate_robot_to_network(document)
    return _robot_to_network_to_pb(document)


def encode_network_to_robot(message: NetworkToRobot) -> network_pb2.NetworkToRobot:
    document = message.to_dict()
    schemas.validate_network_to_robot(document)
    return _network_to_robot_to_pb(document)


def decode_robot_to_network(message: network_pb2.RobotToNetwork) -> RobotToNetwork:
    """Validate an inbound status and turn it into a domain message."""
    document = _robot_to_network_to_doc(message)
    schemas.validate_robot_to_network(document)
    return RobotToNetwork.from_dict(document)


def decode_network_to_robot(message: network_pb2.NetworkToRobot) -> NetworkToRobot:
    """Validate an inbound command and turn it into a domain message."""
    document = _network_to_robot_to_doc(message)
    schemas.validate_network_to_robot(document)
    return NetworkToRobot.from_dict(document)
