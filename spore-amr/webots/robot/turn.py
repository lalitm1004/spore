"""Turning in place, to an absolute heading.

A differential drive can spin about its own axis, so a junction needs no
turning circle: stop on the node, counter-rotate the wheels until the heading
is right, drive on. The caster is frictionless, so nothing resists the spin.

Open-loop dead reckoning is what makes this viable -- there is no line to
follow while turning, so the heading estimate is the only feedback there is.
That estimate was corrected by the marker the robot is standing on, which is
why the turn happens *after* a read and not before.

Pure: no Webots, no I/O.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


def wrap(angle: float) -> float:
    """To [-pi, pi). Turning uses the shortest way round, always.

    Half-open at the negative end, not the positive: exactly pi comes back as
    -pi. The two name the same bearing, so nothing downstream can tell, but the
    range is worth stating correctly for anyone reaching for the boundary.
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class TurnConfig:
    tolerance_rad: float = math.radians(2.0)
    max_rate: float = 4.0     # wheel rad/s at full authority
    min_rate: float = 1.2     # below this the wheels stall rather than creep
    gain: float = 6.0         # wheel rad/s per rad of heading error
    settle_steps: int = 3     # consecutive in-tolerance steps before done
    timeout_s: float = 8.0


class TurnController:
    """Drives heading error to zero by counter-rotating the wheels.

    Proportional, with a floor under the output: a pure P term goes to zero at
    the same time the error does, and the last few degrees are exactly where
    static friction is most able to stop the robot short. The floor keeps
    authority until the tolerance band is actually reached.

    Completion needs several consecutive in-band steps, so a fast pass through
    the target is not mistaken for arriving at it.
    """

    def __init__(self, config: TurnConfig = TurnConfig()):
        self.config = config
        self.target: Optional[float] = None
        self.started_at: Optional[float] = None
        self._in_band = 0

    def start(self, target_rad: float, now: float) -> None:
        self.target = wrap(target_rad)
        self.started_at = now
        self._in_band = 0

    @property
    def active(self) -> bool:
        return self.target is not None

    def error(self, heading_rad: float) -> float:
        if self.target is None:
            return 0.0
        return wrap(self.target - heading_rad)

    def update(self, heading_rad: float, now: float) -> Tuple[float, bool, bool]:
        """Return (steering, done, timed_out).

        `steering` is the differential in wheel rad/s, positive turning left,
        ready to hand to `differential_speeds(base=0, steering=...)`.
        """
        if self.target is None:
            return 0.0, True, False

        config = self.config
        if self.started_at is not None and now - self.started_at > config.timeout_s:
            self.target = None
            return 0.0, False, True

        error = self.error(heading_rad)

        if abs(error) <= config.tolerance_rad:
            self._in_band += 1
            if self._in_band >= config.settle_steps:
                self.target = None
                return 0.0, True, False
            return 0.0, False, False

        self._in_band = 0

        magnitude = min(config.max_rate, max(config.min_rate, abs(error) * config.gain))
        return math.copysign(magnitude, error), False, False
