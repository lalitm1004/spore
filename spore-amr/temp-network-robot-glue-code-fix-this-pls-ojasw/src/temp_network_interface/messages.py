"""Typed domain messages for the robot<->network interface.

These mirror the shared JSON schemas field-for-field. The string-valued fields
(`Mission.type`, `Cargo.state`, `Warning.type`, `Error.type`) are constrained to
the schema's enums at construction, so an invalid value fails immediately rather
than surfacing as a schema error at the wire.

`to_dict()`/`from_dict()` shape the documents exactly as the schemas expect; the
schemas themselves are still validated at the transport boundary, so these
classes are a convenience over the contract, never a divergence from it.

Pure: no grpc, no I/O.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Value sets taken from the schema enums.
MISSION_TYPES = ("PARK", "CHARGE", "HOLD", "IDLE", "CARGO")
CARGO_STATES = ("PICKUP", "DROPOFF", "EN_ROUTE")
WARNING_TYPES = ("LOW_BATTERY", "OBSTACLE")
ERROR_TYPES = ("MOTOR_ERROR", "CAMERA_ERROR", "LIDAR_ERROR", "LOCATION_UNKNOWN", "MISC_ERROR")


@dataclass(frozen=True)
class Battery:
    percentage: float


@dataclass(frozen=True)
class Telemetry:
    battery: Battery

    def to_dict(self) -> dict:
        return {"battery": {"percentage": self.battery.percentage}}

    @classmethod
    def from_dict(cls, doc: dict) -> "Telemetry":
        return cls(battery=Battery(percentage=doc["battery"]["percentage"]))


@dataclass(frozen=True)
class Cargo:
    cargo_id: str
    state: str

    def __post_init__(self):
        if self.state not in CARGO_STATES:
            raise ValueError("unknown cargo state {!r}; expected one of {}".format(
                self.state, ", ".join(CARGO_STATES)))

    def to_dict(self) -> dict:
        return {"cargo_id": self.cargo_id, "state": self.state}

    @classmethod
    def from_dict(cls, doc: dict) -> "Cargo":
        return cls(cargo_id=doc["cargo_id"], state=doc["state"])


@dataclass(frozen=True)
class Mission:
    type: str
    cargo: Optional[Cargo] = None

    def __post_init__(self):
        if self.type not in MISSION_TYPES:
            raise ValueError("unknown mission type {!r}; expected one of {}".format(
                self.type, ", ".join(MISSION_TYPES)))
        if self.type == "CARGO" and self.cargo is None:
            raise ValueError("a CARGO mission requires a cargo")

    def to_dict(self) -> dict:
        doc = {"type": self.type}
        if self.cargo is not None:
            doc["cargo"] = self.cargo.to_dict()
        return doc

    @classmethod
    def from_dict(cls, doc: dict) -> "Mission":
        cargo = Cargo.from_dict(doc["cargo"]) if "cargo" in doc else None
        return cls(type=doc["type"], cargo=cargo)


@dataclass(frozen=True)
class Warning:
    type: str
    percentage: Optional[float] = None
    current_node_id: Optional[int] = None

    def __post_init__(self):
        if self.type not in WARNING_TYPES:
            raise ValueError("unknown warning type {!r}; expected one of {}".format(
                self.type, ", ".join(WARNING_TYPES)))

    def to_dict(self) -> dict:
        if self.type == "LOW_BATTERY":
            return {"type": "LOW_BATTERY", "percentage": self.percentage}
        return {"type": "OBSTACLE", "current_node_id": self.current_node_id}

    @classmethod
    def from_dict(cls, doc: dict) -> "Warning":
        if doc["type"] == "LOW_BATTERY":
            return cls(type="LOW_BATTERY", percentage=doc["percentage"])
        return cls(type="OBSTACLE", current_node_id=doc["current_node_id"])


@dataclass(frozen=True)
class Error:
    type: str

    def __post_init__(self):
        if self.type not in ERROR_TYPES:
            raise ValueError("unknown error type {!r}; expected one of {}".format(
                self.type, ", ".join(ERROR_TYPES)))

    def to_dict(self) -> dict:
        return {"type": self.type}

    @classmethod
    def from_dict(cls, doc: dict) -> "Error":
        return cls(type=doc["type"])


@dataclass(frozen=True)
class Fault:
    warning: Optional[Warning] = None
    error: Optional[Error] = None

    def to_dict(self) -> dict:
        doc = {}
        if self.warning is not None:
            doc["warning"] = self.warning.to_dict()
        if self.error is not None:
            doc["error"] = self.error.to_dict()
        return doc

    @classmethod
    def from_dict(cls, doc: dict) -> "Fault":
        return cls(
            warning=Warning.from_dict(doc["warning"]) if "warning" in doc else None,
            error=Error.from_dict(doc["error"]) if "error" in doc else None,
        )


@dataclass(frozen=True)
class RobotToNetwork:
    bot_id: int
    region_id: int
    latest_node_id: int
    mission: Mission
    telemetry: Telemetry
    timestamp: int
    fault: Optional[Fault] = None

    def to_dict(self) -> dict:
        doc = {
            "bot_id": self.bot_id,
            "region_id": self.region_id,
            "latest_node_id": self.latest_node_id,
            "mission": self.mission.to_dict(),
            "telemetry": self.telemetry.to_dict(),
            "timestamp": self.timestamp,
        }
        if self.fault is not None:
            doc["fault"] = self.fault.to_dict()
        return doc

    @classmethod
    def from_dict(cls, doc: dict) -> "RobotToNetwork":
        return cls(
            bot_id=doc["bot_id"],
            region_id=doc["region_id"],
            latest_node_id=doc["latest_node_id"],
            mission=Mission.from_dict(doc["mission"]),
            telemetry=Telemetry.from_dict(doc["telemetry"]),
            timestamp=doc["timestamp"],
            fault=Fault.from_dict(doc["fault"]) if "fault" in doc else None,
        )


@dataclass(frozen=True)
class NetworkToRobot:
    target_node_id: int
    timestamp: int
    set_mission: Optional[Mission] = None

    def to_dict(self) -> dict:
        doc = {"target_node_id": self.target_node_id, "timestamp": self.timestamp}
        if self.set_mission is not None:
            doc["set_mission"] = self.set_mission.to_dict()
        return doc

    @classmethod
    def from_dict(cls, doc: dict) -> "NetworkToRobot":
        return cls(
            target_node_id=doc["target_node_id"],
            timestamp=doc["timestamp"],
            set_mission=Mission.from_dict(doc["set_mission"]) if "set_mission" in doc else None,
        )


@dataclass(frozen=True)
class RobotState:
    """The whole of a robot's knowledge, as last conveyed by the network.

    The firmware knows its own target and mission and nothing else -- the fleet
    is the network layer's concern, never the firmware's. Each `NetworkToRobot`
    command replaces this state, which the client preserves until the next
    command arrives, so the robot keeps its goal even between commands.
    """

    target_node_id: Optional[int] = None
    mission: Optional[Mission] = None
    timestamp: Optional[int] = None

    @classmethod
    def from_command(cls, command: NetworkToRobot) -> "RobotState":
        return cls(
            target_node_id=command.target_node_id,
            mission=command.set_mission,
            timestamp=command.timestamp,
        )
