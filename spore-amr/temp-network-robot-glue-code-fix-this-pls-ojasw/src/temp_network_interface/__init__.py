"""The robot <-> network gRPC interface, following the shared JSON schemas.

Two directions, one contract each:

    robot-to-network.schema.json   -- a robot reports status upward
    network-to-robot.schema.json   -- the network commands a robot

The schemas are the ground-truth interface; the gRPC service (`proto/network.proto`)
is an opaque envelope that ships each payload as validated JSON. This split means
the schemas can evolve without touching the transport, and a network layer written
in any language can validate against the same files.

Importing this package pulls only the pure domain model, world state, durability
and relay -- no grpc. The grpc-touching pieces -- `transport`, `client`, `server`
-- are imported explicitly by whoever uses them, so the typed core stays
host-testable without grpc installed (the same boundary the webots implementation
draws around the Webots API).
"""

from temp_network_interface.messages import (
    Battery,
    Cargo,
    Error,
    Fault,
    Mission,
    NetworkToRobot,
    RobotState,
    RobotToNetwork,
    Telemetry,
    Warning,
)
from temp_network_interface.policy import HoldPolicy, NoopPolicy, Policy
from temp_network_interface.relay import Relay
from temp_network_interface.schemas import (
    NETWORK_TO_ROBOT,
    ROBOT_TO_NETWORK,
    validate_network_to_robot,
    validate_robot_to_network,
)
from temp_network_interface.state import Fleet, TargetedCommand, fulfilled
from temp_network_interface.store import Journal

__all__ = [
    "Battery",
    "Cargo",
    "Error",
    "Fault",
    "Mission",
    "NetworkToRobot",
    "RobotState",
    "RobotToNetwork",
    "Telemetry",
    "Warning",
    "Fleet",
    "TargetedCommand",
    "fulfilled",
    "Journal",
    "Relay",
    "Policy",
    "HoldPolicy",
    "NoopPolicy",
    "NETWORK_TO_ROBOT",
    "ROBOT_TO_NETWORK",
    "validate_network_to_robot",
    "validate_robot_to_network",
]
