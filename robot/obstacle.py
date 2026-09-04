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
from typing import Optional, Sequence


class Obstacle(Enum):
    CLEAR = "CLEAR"      # nothing ahead; drive normally
    BACKING = "BACKING"  # reversing, back toward the last node
    HOLDING = "HOLDING"  # parked at the node, waiting for it to go away


@dataclass(frozen=True)
class ObstacleConfig:
    stop_m: float = 0.18      # closer than this and the reflex fires
    clear_m: float = 0.30     # must be further than this to resume
    arrive_m: float = 0.04    # close enough to count as back at the node
    max_backoff_m: float = 2.0   # give up reversing after this much travel
    backoff_speed: float = 2.0   # wheel rad/s, gentle
    departed_m: float = 0.15  # range must improve by this much, while parked,
                              # before the obstacle counts as gone

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
        self.retreating = False
        self.parked_range: Optional[float] = None
        self.trips = 0
        self.last_range = float("inf")

    def update(self, nearest_m: float, travelled: float,
               retreat_remaining: Optional[float] = None) -> Obstacle:
        """Advance the reflex.

        `travelled` is how far the robot has moved since the reflex fired, in
        metres, used only as a give-up bound. `retreat_remaining` is how much
        of the path back to the last node is left, or None if no node has been
        read yet.

        The reflex retreats to a node rather than to some arbitrary clearance,
        because a node is a position the router can act on -- stopping 80 mm
        back leaves the robot mid-lane with nothing useful to say about where
        it is. Straight-line distance will not do for that: the lane curves, so
        reversing straight diverges from the node instead of returning to it.
        The caller measures the remaining path from the wheels, which retrace
        exactly.
        """
        self.last_range = nearest_m
        config = self.config

        if self.state is Obstacle.CLEAR:
            if nearest_m < config.stop_m:
                self.state = Obstacle.BACKING
                self.retreating = retreat_remaining is not None
                self.trips += 1
            return self.state

        if self.state is Obstacle.BACKING:
            if self.retreating and retreat_remaining is not None:
                if retreat_remaining <= config.arrive_m:
                    self.state = Obstacle.HOLDING
                    self.parked_range = None
                    return self.state
            elif nearest_m >= config.clear_m:
                # No node read yet, so there is nowhere to retreat to; settle
                # for clearance.
                self.state = Obstacle.HOLDING
                self.parked_range = None
                return self.state

            if travelled >= config.max_backoff_m:
                # Reversing is not helping -- stop rather than keep going
                # blind, since there is no rear sensor.
                self.state = Obstacle.HOLDING
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
            self.state = Obstacle.CLEAR
            self.backing_from = None
            self.retreating = False
            self.parked_range = None
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
