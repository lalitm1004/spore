"""Analytic centerlines for generated tracks.

A centerline knows its own geometry, so telemetry can compute true
cross-track error independently of what the robot's sensors report.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Oval:
    """A stadium: two straights joined by semicircular ends."""

    width: float
    height: float

    @property
    def radius(self) -> float:
        return self.height / 2.0

    @property
    def straight(self) -> float:
        return self.width - self.height

    @property
    def length(self) -> float:
        return 2 * self.straight + 2 * math.pi * self.radius

    def point_at(self, s: float) -> Tuple[float, float]:
        """Point at normalised arclength `s`, wrapping outside [0, 1)."""
        radius, straight = self.radius, self.straight
        arc = math.pi * radius
        travelled = (s % 1.0) * self.length

        if travelled <= straight:  # bottom straight, heading +x
            return (-straight / 2 + travelled, -radius)

        travelled -= straight
        if travelled <= arc:  # right semicircle, counter-clockwise
            theta = -math.pi / 2 + travelled / radius
            return (straight / 2 + radius * math.cos(theta), radius * math.sin(theta))

        travelled -= arc
        if travelled <= straight:  # top straight, heading -x
            return (straight / 2 - travelled, radius)

        travelled -= straight  # left semicircle
        theta = math.pi / 2 + travelled / radius
        return (-straight / 2 + radius * math.cos(theta), radius * math.sin(theta))

    def signed_distance_to(self, x: float, y: float) -> float:
        """Cross-track error at (x, y): negative inside the loop, positive outside.

        A stadium is exactly the locus of points at `radius` from its central
        spine segment, so the distance to the curve is the distance to that
        segment minus the radius.
        """
        half = self.straight / 2
        nearest_x = max(-half, min(half, x))
        return math.hypot(x - nearest_x, y) - self.radius


def oval(width: float, height: float) -> Oval:
    return Oval(width=width, height=height)
