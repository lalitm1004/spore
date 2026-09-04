"""PID controller. Pure: no Webots, no I/O."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ControlOutput:
    u: float
    p: float
    i: float
    d: float


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: Optional[float] = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error: Optional[float] = None

    def update(self, error: float, dt: float) -> ControlOutput:
        previous_integral = self._integral
        self._integral += error * dt

        if self._previous_error is None:
            derivative = 0.0  # no derivative kick on the first step
        else:
            derivative = (error - self._previous_error) / dt
        self._previous_error = error

        p = self.kp * error
        i = self.ki * self._integral
        d = self.kd * derivative
        u = p + i + d

        if self.output_limit is not None and abs(u) > self.output_limit:
            # Conditional integration: while saturated, only let the integral
            # move in the direction that brings the output back into range.
            if _same_sign(u, error):
                self._integral = previous_integral
                i = self.ki * self._integral
                u = p + i + d
            u = max(-self.output_limit, min(self.output_limit, u))

        return ControlOutput(u=u, p=p, i=i, d=d)


def _same_sign(a: float, b: float) -> bool:
    return (a >= 0) == (b >= 0)
