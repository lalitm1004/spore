"""Distance travelled, from wheel encoders.

The firmware needs this to cross a marker blind: a marker tile covers the line
it was following, so for ~115 mm it has no lateral feedback and must hold its
heading on dead reckoning alone.

On hardware these are quadrature encoders and the one genuine place for
pin-change interrupts. In Webots they are `PositionSensor`s reporting shaft
angle in radians. Pure: no Webots, no I/O.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Pose:
    """Dead-reckoned pose in the odometry frame, metres and radians."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    distance: float = 0.0  # total path length, monotonic


class Odometry:
    """Integrates wheel angles into a pose.

    Tracks `distance` separately from position because that is what marker
    crossing keys off -- how far have I gone since the trigger, not where am I.
    Absolute position drifts; a 115 mm arc length does not, much.
    """

    def __init__(self, wheel_radius: float, track_width: float):
        if wheel_radius <= 0 or track_width <= 0:
            raise ValueError("wheel radius and track width must be positive")
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self._last: Optional[Tuple[float, float]] = None
        self.pose = Pose()

    def reset(self, pose: Pose = Pose()) -> None:
        """Re-zero the pose, keeping the encoder reference.

        Used when a marker read gives an absolute fix: the QR's payload is
        ground truth, so drift accumulated getting there can be discarded.
        """
        self.pose = pose

    def update(self, left_angle: float, right_angle: float) -> Pose:
        """Fold in new encoder readings and return the updated pose.

        The first call only establishes a reference -- a shaft angle is
        meaningless in isolation, only its change carries distance.
        """
        if self._last is None:
            self._last = (left_angle, right_angle)
            return self.pose

        last_left, last_right = self._last
        left = (left_angle - last_left) * self.wheel_radius
        right = (right_angle - last_right) * self.wheel_radius
        self._last = (left_angle, right_angle)

        forward = (left + right) / 2.0
        turn = (right - left) / self.track_width

        # Midpoint heading over the step: exact for a straight line, and
        # second-order for an arc, which is far more than 16 ms needs.
        heading = self.pose.theta + turn / 2.0
        self.pose = Pose(
            x=self.pose.x + forward * math.cos(heading),
            y=self.pose.y + forward * math.sin(heading),
            theta=_wrap(self.pose.theta + turn),
            distance=self.pose.distance + abs(forward),
        )
        return self.pose


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
