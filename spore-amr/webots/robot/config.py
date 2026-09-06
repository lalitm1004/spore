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
    # What the robot does when it is carrying something. An empty robot runs at
    # `base_speed`; a loaded one drops to this. Defaults to `base_speed`, so a
    # config that says nothing behaves exactly as it did before.
    laden_speed: float = None
    max_speed: float = 20.0
    # How hard the robot may accelerate, in rad/s per second at the wheels.
    # Deceleration is never limited -- stopping is a safety action and waiting
    # on a ramp to stop is the wrong trade.
    #
    # This exists because speed and turn accuracy are coupled. Coming out of a
    # turn the robot carries a few degrees of heading error, and lateral drift
    # is v*sin(e) against 10 mm of line: at 0.12 m/s the PID has 1.19 s to pull
    # it back, at 0.36 m/s only 0.40 s. Measured at 18 rad/s with no ramp, two
    # of eight robots lost the line immediately after a turn and halted.
    # Starting from rest after every turn gives the loop its second back
    # without giving up speed on the straight.
    accel_rad_s2: float = 12.0
    steering_limit: float = 6.0
    lost_line_timeout_s: float = 2.0
    # How long the line must actually be gone before the firmware reports it.
    # A 16 ms dropout is noise -- three ticks after a crossing ends, the array
    # is not always back over the lane yet -- and the companion answers every
    # report by cutting speed, so undebounced blips ratcheted robots down to
    # the floor. Well under `lost_line_timeout_s`, which handles real losses.
    lost_line_debounce_s: float = 0.1
    # And the way back up: this much clean line wins back one speed step.
    speed_recover_after_s: float = 5.0
    # How long to hold a junction waiting for the network layer before giving
    # up and carrying straight on. A robot that is never answered must not
    # block a lane for the rest of the run.
    junction_timeout_s: float = 6.0
    # How long the robot stands still to load or unload cargo. Collecting and
    # delivering were instantaneous -- the reports *were* the handling, because
    # there is no manipulator to simulate -- so a robot arrived at a transfer
    # node and left in the same breath. Nothing in the fleet was wrong about it
    # and it looks wrong, because a real AMR takes time at a station and a
    # warehouse's throughput depends on how long.
    #
    # Well under `junction_timeout_s`: the robot is holding, not waiting to be
    # told anything, so nothing upstream should time out while it works.
    cargo_handling_s: float = 10.0
    # Sim seconds to sit still before leaving the start node. The fleet is
    # released one robot at a time so bay-mates do not reach their shared
    # junction together; the firmware holds this itself rather than waiting to
    # be told, because the companion attaches some time after boot and a robot
    # that moved in the meantime has already left its bay.
    start_delay_s: float = 0.0
    turn_tolerance_deg: float = 2.0
    turn_rate: float = 4.0
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
    max_backoff_m: float = 0.45
    departed_m: float = 0.15
    hold_timeout_s: float = 8.0
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
    # The heading the robot is placed on, in world radians. A `TURN` carries an
    # absolute bearing off the map and the turn controller's only feedback is
    # this odometry, so the two must share a frame. Booting at 0 regardless of
    # placement put every robot on the warehouse window out by its bay's
    # bearing -- +/-90 degrees, measured against the supervisor -- and turned
    # every junction onto the wrong lane. A docked robot knows which way its
    # bay faces; that is commissioning data, not a sensor.
    start_theta: float = 0.0


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
