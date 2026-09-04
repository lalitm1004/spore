"""Per-robot controller configuration. Pure: no Webots, no I/O beyond parsing."""

from dataclasses import dataclass, field
from typing import Tuple


@dataclass(frozen=True)
class PIDGains:
    kp: float = 0.9
    ki: float = 0.0
    kd: float = 0.04


@dataclass(frozen=True)
class SensorConfig:
    offsets: Tuple[float, ...]
    white_ref: float = 1000.0
    black_ref: float = 200.0
    min_confidence: float = 0.15

    def __post_init__(self):
        if self.white_ref <= self.black_ref:
            raise ValueError(
                "white_ref ({}) must be greater than black_ref ({})".format(
                    self.white_ref, self.black_ref
                )
            )
        if not self.offsets:
            raise ValueError("at least one sensor offset is required")


@dataclass(frozen=True)
class ControlConfig:
    base_speed: float = 4.0
    max_speed: float = 20.0
    steering_limit: float = 6.0
    lost_line_timeout_s: float = 2.0
    pid: PIDGains = field(default_factory=PIDGains)


@dataclass(frozen=True)
class ControllerConfig:
    name: str
    sensors: SensorConfig
    control: ControlConfig

    @classmethod
    def from_dict(cls, data: dict) -> "ControllerConfig":
        sensors = dict(data["sensors"])
        sensors["offsets"] = tuple(float(o) for o in sensors["offsets"])

        control = dict(data.get("control") or {})
        control["pid"] = PIDGains(**(control.get("pid") or {}))

        return cls(
            name=data["name"],
            sensors=SensorConfig(**sensors),
            control=ControlConfig(**control),
        )
