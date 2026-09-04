"""Forward obstacle reflex: stop, back off, hold until it clears.

Deliberately a reflex and not a planner. It reads one sensor, it can only stop
or reverse, and nothing it sees ever becomes a route decision -- routing is the
network layer's job and stays that way. That separation is what keeps the
"collision avoidance is a property of the protocol, not a sensor" argument
intact even with a range sensor on the robot: this exists to survive the case
the protocol cannot cover, which is something that was never announced because
it is not a robot.

Hysteresis is the point of the two thresholds. A single one chatters at the
boundary -- stop, clear, stop, clear -- which on real hardware is how you burn
out a gearbox.

Pure: no Webots, no I/O.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class Obstacle(Enum):
    CLEAR = "CLEAR"      # nothing ahead; drive normally
    BACKING = "BACKING"  # reversing away from what was seen
    HOLDING = "HOLDING"  # backed off, waiting for it to go away


@dataclass(frozen=True)
class ObstacleConfig:
    stop_m: float = 0.18     # closer than this and the reflex fires
    clear_m: float = 0.30    # must be further than this to resume
    max_backoff_m: float = 0.15  # give up reversing after this much travel
    backoff_speed: float = 2.0   # wheel rad/s, gentle

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
    """Tracks the reflex across steps, driven by odometry distance."""

    def __init__(self, config: ObstacleConfig = ObstacleConfig()):
        self.config = config
        self.state = Obstacle.CLEAR
        self.backing_from = None
        self.trips = 0
        self.last_range = float("inf")

    def update(self, nearest_m: float, x: float, y: float) -> Obstacle:
        """Advance the reflex. `x`, `y` are the odometry pose, in metres."""
        self.last_range = nearest_m
        config = self.config

        if self.state is Obstacle.CLEAR:
            if nearest_m < config.stop_m:
                self.state = Obstacle.BACKING
                self.backing_from = (x, y)
                self.trips += 1
            return self.state

        if self.state is Obstacle.BACKING:
            # Reverse until there is actually clearance -- the goal is distance
            # from the obstacle, so measure that rather than a proxy for it.
            #
            # An earlier version counted odometry path length instead, and path
            # length is monotonic: slamming from +6 to -2 rad/s leaves the robot
            # coasting forward for a few steps, and that overshoot counted as
            # progress. The reflex finished having driven partly INTO the thing
            # it was backing away from.
            if nearest_m >= config.clear_m:
                self.state = Obstacle.HOLDING
                return self.state

            start = self.backing_from
            if start is not None:
                travelled = ((x - start[0]) ** 2 + (y - start[1]) ** 2) ** 0.5
                if travelled >= config.max_backoff_m:
                    # Reversing is not helping -- stop rather than keep going
                    # blind, since there is no rear sensor.
                    self.state = Obstacle.HOLDING
            return self.state

        # HOLDING: only a clear path well beyond the trip point resumes.
        if nearest_m > config.clear_m:
            self.state = Obstacle.CLEAR
            self.backing_from = None
        return self.state

    @property
    def blocked(self) -> bool:
        """True whenever the reflex owns the motors."""
        return self.state is not Obstacle.CLEAR

    def speeds(self) -> float:
        """Base wheel speed the reflex wants. Negative reverses."""
        if self.state is Obstacle.BACKING:
            return -self.config.backoff_speed
        return 0.0

    @property
    def clearance(self) -> float:
        """Last range reading, for telemetry and events."""
        return self.last_range
