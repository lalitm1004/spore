"""Forward obstacle reflex: slow, settle, reverse to the last marker, hold.

Deliberately a reflex and not a planner. It reads one sensor, it can only slow,
stop and reverse, and nothing it sees ever becomes a route decision -- routing
is the network layer's job and stays that way. That separation is what keeps
"collision avoidance is a property of the protocol, not a sensor" true even
with a range finder aboard: this covers the one case the protocol cannot,
which is something that never announced itself because it is not a robot.

The sequence is deliberately unhurried:

    CLEAR -> STOPPING -> PAUSED -> BACKING -> HOLDING

Going straight from cruise into reverse pitches the chassis hard enough to
throw the camera boom around, and on real hardware is how a gearbox dies.
Ramping down, settling, then ramping into reverse costs about two seconds and
is what the hardware would have to do anyway.

Ending the retreat is the colour sensor's job, not odometry's. Reversing over a
marker, the sensor crosses the tile's far orange band, then the code, then the
near band; that second band is the tile's near edge, which is where the robot
stood before it drove on. Counting bands needs no dead reckoning, so it cannot
drift, and it does not care that the lane curves.

Pure: no Webots, no I/O.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence


class Obstacle(Enum):
    CLEAR = "CLEAR"        # nothing ahead; drive normally
    STOPPING = "STOPPING"  # ramping down to a halt
    PAUSED = "PAUSED"      # stopped, settling before reversing
    BACKING = "BACKING"    # reversing toward the last marker
    HOLDING = "HOLDING"    # parked, waiting for it to go away


@dataclass(frozen=True)
class ObstacleConfig:
    stop_m: float = 0.18       # closer than this and the reflex fires
    clear_m: float = 0.30      # far enough to stop reversing, absent a marker
    decel_s: float = 0.8       # ramp from cruise down to a halt
    pause_s: float = 1.0       # settle before reversing
    accel_s: float = 0.6       # ramp into reverse
    backoff_speed: float = 2.0     # wheel rad/s once fully reversing
    borders_to_pass: int = 2   # orange bands to cross before stopping
    # Give up reversing after this much travel. Well under a lane length on
    # purpose: at 2.0 m a robot could reverse the entire span it came down and
    # keep going off the edge of the floor, which is what ten robots tripping
    # over each other actually did. Clearing a blocked lane needs centimetres,
    # not metres -- if two orange bands have not appeared in half a metre, the
    # marker is not behind us and reversing further is driving blind.
    max_backoff_m: float = 0.45
    departed_m: float = 0.15   # range must improve by this much, while parked,
                               # before the obstacle counts as gone
    hold_timeout_s: float = 8.0    # then try again anyway -- see below

    def __post_init__(self):
        if self.clear_m <= self.stop_m:
            raise ValueError(
                "clear_m ({}) must exceed stop_m ({}) or the reflex will "
                "chatter at the threshold".format(self.clear_m, self.stop_m))


def nearest(ranges: Sequence[float], max_range: float) -> float:
    """Closest valid return in the scan.

    A lidar reports its max range for "nothing there", and some drivers report
    inf. Both mean the same thing and neither should win a min().
    """
    valid = [r for r in ranges
             if r is not None and r == r and 0.0 < r < max_range * 0.999]
    return min(valid) if valid else float("inf")


class ObstacleGuard:
    """Tracks the reflex across steps."""

    def __init__(self, config: ObstacleConfig = ObstacleConfig()):
        self.config = config
        self.state = Obstacle.CLEAR
        self.entered_at: Optional[float] = None
        self.cruise_speed = 0.0
        self.borders_seen = 0
        self.trips = 0
        self.timeouts = 0
        self.last_range = float("inf")
        self.parked_range: Optional[float] = None
        self._border_was = False

    # ------------------------------------------------------------ stepping --

    def update(self, nearest_m: float, now: float, sees_border: bool,
               travelled: float, cruise_speed: float = 0.0) -> Obstacle:
        """Advance the reflex.

        `travelled` is metres since the reflex fired, used only as a give-up
        bound. `cruise_speed` is what the wheels were doing when it tripped, so
        the deceleration ramp starts from the right place rather than jumping.
        """
        self.last_range = nearest_m
        config = self.config
        elapsed = now - (self.entered_at if self.entered_at is not None else now)

        if self.state is Obstacle.CLEAR:
            if nearest_m < config.stop_m:
                self._enter(Obstacle.STOPPING, now)
                self.cruise_speed = cruise_speed
                self.trips += 1
            return self.state

        if self.state is Obstacle.STOPPING:
            if elapsed >= config.decel_s:
                self._enter(Obstacle.PAUSED, now)
            return self.state

        if self.state is Obstacle.PAUSED:
            if elapsed >= config.pause_s:
                self._enter(Obstacle.BACKING, now)
                self.borders_seen = 0
                # If the reflex fired while the sensor was still over a tile,
                # that band is one the robot has already crossed. Seed the edge
                # detector with it so its trailing edge is not counted as an
                # arrival.
                self._border_was = sees_border
            return self.state

        if self.state is Obstacle.BACKING:
            if sees_border and not self._border_was:
                self.borders_seen += 1
            self._border_was = sees_border

            if self.borders_seen >= config.borders_to_pass:
                self._enter(Obstacle.HOLDING, now)
                self.parked_range = None
            elif travelled >= config.max_backoff_m:
                # Reversing is not finding the marker and there is no rear
                # sensor, so stop rather than keep going blind.
                self._enter(Obstacle.HOLDING, now)
                self.parked_range = None
            return self.state

        # HOLDING. Resuming on `nearest_m > clear_m` alone livelocks: reversing
        # is itself what produced the clearance, so the robot drives forward,
        # trips on the same obstacle, reverses, and repeats for ever.
        #
        # The robot is stationary here, so a range that improves further can
        # only mean the obstacle itself moved. That is the signal to trust.
        if self.parked_range is None:
            self.parked_range = nearest_m
        elif nearest_m > self.parked_range + config.departed_m:
            self._enter(Obstacle.CLEAR, now)
            self.parked_range = None
            self.borders_seen = 0
        elif elapsed >= config.hold_timeout_s:
            # Waiting for the obstacle to move works when the obstacle is a
            # carton. It does not when the obstacle is another robot that is
            # also waiting: neither moves, and the pair is deadlocked. Measured
            # with ten robots and no coordination -- one spent 69% of a run
            # parked behind another.
            #
            # Retrying does not resolve the conflict; that is the network
            # layer's job, and this reflex has no business trying. It only
            # stops a jam from being permanent, and the retry trips straight
            # back if the way is still blocked.
            self.timeouts += 1
            self._enter(Obstacle.CLEAR, now)
            self.parked_range = None
            self.borders_seen = 0
        return self.state

    def _enter(self, state: Obstacle, now: float) -> None:
        self.state = state
        self.entered_at = now

    # ------------------------------------------------------------- outputs --

    @property
    def blocked(self) -> bool:
        """True whenever the reflex owns the motors."""
        return self.state is not Obstacle.CLEAR

    def speeds(self, now: float) -> float:
        """Base wheel speed the reflex wants, ramped. Negative reverses."""
        config = self.config
        elapsed = now - (self.entered_at if self.entered_at is not None else now)

        if self.state is Obstacle.STOPPING:
            if config.decel_s <= 0:
                return 0.0
            return self.cruise_speed * max(0.0, 1.0 - elapsed / config.decel_s)

        if self.state is Obstacle.BACKING:
            if config.accel_s <= 0:
                return -config.backoff_speed
            return -config.backoff_speed * min(1.0, elapsed / config.accel_s)

        return 0.0

    @property
    def clearance(self) -> float:
        """Last range reading, for telemetry and events."""
        return self.last_range
