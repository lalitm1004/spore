"""Per-robot controller configuration. Pure: no Webots, no I/O beyond parsing."""

from dataclasses import dataclass, field
from typing import Tuple

from robot.hal import Adc


@dataclass(frozen=True)
class PIDGains:
    kp: float = 0.9
    ki: float = 0.0
    kd: float = 0.04


@dataclass(frozen=True)
class SensorConfig:
    offsets: Tuple[float, ...]
    # References are in ADC counts: 1000 raw -> 1023, 200 raw -> 205.
    white_ref: float = 1023.0
    black_ref: float = 205.0
    min_confidence: float = 0.15
    sample_period_s: float = 0.016
    latency_s: float = 0.0
    adc: Adc = field(default_factory=Adc)

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
class OpticsConfig:
    """The colour trigger and the QR camera, and the tile they read.

    Geometry is duplicated from the PROTO because the firmware cannot ask
    Webots where its own sensors are mounted -- on hardware it could not
    either. `tools/gen_fleet.py` writes both from `fleet.yaml`, so they cannot
    drift apart.
    """

    enabled: bool = True
    color_sensor_x: float = 0.125
    ir_array_x: float = 0.070
    camera_x: float = 0.095
    camera_footprint: float = 0.0927
    tile_length: float = 0.100
    code_size: float = 0.060
    border_rgb: Tuple[int, int, int] = (255, 122, 0)
    border_tolerance: float = 0.30
    crossing_speed: float = 0.0   # 0 keeps cruise speed; >0 slows for the read


@dataclass(frozen=True)
class LidarConfig:
    """The forward obstacle reflex.

    A reflex, not a planner: it may stop and reverse, and nothing it sees
    reaches the router. `clear_m` exceeds `stop_m` on purpose -- one threshold
    chatters at the boundary.
    """

    enabled: bool = True
    stop_m: float = 0.18
    clear_m: float = 0.30
    decel_s: float = 0.8
    pause_s: float = 1.0
    accel_s: float = 0.6
    borders_to_pass: int = 2
    max_backoff_m: float = 2.0
    departed_m: float = 0.15
    backoff_speed: float = 2.0
    max_range: float = 1.0


@dataclass(frozen=True)
class OdometryConfig:
    """Wheel geometry, as the odometry believes it.

    `track_width` is calibrated, not measured off the model. The wheels are
    45 mm from centre, but a run of four markers showed odometry over-reporting
    rotation by 10.5% against ground truth, consistently to within 0.5% --
    contact behaves like the wheels' outer edge at 50 mm, not their centre.
    Believing the nominal 90 mm cost 27 degrees of heading drift per lap.

    Recalibrate with `robot/supervisor.py --calibrate` after any change to the
    wheels or the contact material.
    """

    wheel_radius: float = 0.020
    track_width: float = 0.0994


@dataclass(frozen=True)
class ControllerConfig:
    name: str
    sensors: SensorConfig
    control: ControlConfig
    optics: OpticsConfig = field(default_factory=OpticsConfig)
    odometry: OdometryConfig = field(default_factory=OdometryConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)

    @classmethod
    def from_dict(cls, data: dict) -> "ControllerConfig":
        sensors = dict(data["sensors"])
        sensors["offsets"] = tuple(float(o) for o in sensors["offsets"])
        if "adc" in sensors:
            sensors["adc"] = Adc(**sensors["adc"])

        control = dict(data.get("control") or {})
        control["pid"] = PIDGains(**(control.get("pid") or {}))

        optics = dict(data.get("optics") or {})
        if "border_rgb" in optics:
            optics["border_rgb"] = tuple(int(c) for c in optics["border_rgb"])

        return cls(
            name=data["name"],
            sensors=SensorConfig(**sensors),
            control=ControlConfig(**control),
            optics=OpticsConfig(**optics),
            odometry=OdometryConfig(**(data.get("odometry") or {})),
            lidar=LidarConfig(**(data.get("lidar") or {})),
        )
