"""How long a manoeuvre takes and what it costs in energy.

The timing model is decomposed so that nothing is counted twice. The naive
approach -- "time to cross an edge from rest" plus "time to stop at the end" --
double-counts the distance covered while accelerating and decelerating. Instead
every traversal is priced as if the robot cruises the whole edge, and stopping is
priced as the *extra* time a stop costs relative to cruising through:

    to decelerate from v to 0 and accelerate back to v, a robot covers v^2/a in
    time 2v/a; cruising that same distance takes v/a; so a full stop-and-go costs
    v/a of extra time, wherever it happens.

That gives four primitives -- cruise, stop-and-go, half of one (starting from rest
or stopping for good), and rotation -- which compose without overlap:

    hop = cruise(edge) + (stop_and_go + turn(q) + wait  if the robot stops here)

Rotation is in place, so a robot that turns has necessarily stopped; a robot that
waits has too. It stops once, not once per reason.

Every constant here is a placeholder until real hardware numbers exist. They are
chosen to be self-consistent for a ~100 kg AMR at 1 m/s, and the relationships that
matter to the planner's behaviour are: moving draws several times more power than
idling (so waiting beats detouring when energy is scarce), and a turn costs
meaningfully more than a straight pass (so straight runs are preferred).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


class Kinematics(Protocol):
    """Timing and energy for one robot model."""

    def cruise_ms(self, distance_cm: int) -> int: ...

    def stop_and_go_ms(self) -> int: ...

    def half_stop_ms(self) -> int: ...

    def turn_ms(self, quarter_turns: int) -> int: ...

    def cruise_energy_j(self, distance_cm: int) -> float: ...

    def stop_and_go_energy_j(self) -> float: ...

    def turn_energy_j(self, quarter_turns: int) -> float: ...

    def idle_energy_j(self, duration_ms: int) -> float: ...


@dataclass(frozen=True, slots=True)
class RobotKinematics:
    """Trapezoidal-profile kinematics with a simple energy model.

    Times are integer milliseconds, rounded up: the planner's output becomes a
    reservation window, and a window that is too short is unsafe, whereas one that
    is a millisecond too long merely costs a sliver of throughput.
    """

    cruise_speed_cm_s: float = 100.0
    """Steady travel speed. 1 m/s is typical for a loaded warehouse AMR."""

    accel_cm_s2: float = 60.0
    """Acceleration, and by symmetry deceleration."""

    turn_ms_per_quarter: int = 900
    """Time to rotate 90 degrees in place."""

    rolling_j_per_cm: float = 2.0
    """Energy to travel one cm at cruise -- about 200 W at 1 m/s."""

    stop_and_go_j: float = 80.0
    """Energy thrown away by one full stop and restart (~0.5*m*v^2 plus losses)."""

    turn_j_per_quarter: float = 120.0
    """Energy for a 90 degree rotation in place."""

    idle_w: float = 30.0
    """Standing power draw. Well under the moving draw, which is what makes waiting
    a cheaper answer than detouring when the battery is low."""

    def __post_init__(self) -> None:
        if self.cruise_speed_cm_s <= 0:
            raise ValueError("cruise_speed_cm_s must be positive")
        if self.accel_cm_s2 <= 0:
            raise ValueError("accel_cm_s2 must be positive")

    # -- geometry guard ------------------------------------------------------

    @property
    def stop_and_go_distance_cm(self) -> float:
        """Distance covered decelerating to a halt and getting back to cruise."""
        return self.cruise_speed_cm_s**2 / self.accel_cm_s2

    def validate_for_spacing(self, node_spacing_cm: int) -> None:
        """Check the trapezoidal assumption holds for this map's edge length.

        The decomposition above assumes the robot actually reaches cruise speed
        within one edge. If the edges are shorter than the stop-and-go distance it
        never does, and every traversal time here would be optimistic -- which
        would silently under-size reservations. Fail loudly instead.
        """
        if self.stop_and_go_distance_cm > node_spacing_cm:
            raise ValueError(
                f"a stop and restart covers {self.stop_and_go_distance_cm:.0f} cm, "
                f"which exceeds the {node_spacing_cm} cm between nodes: the robot "
                "never reaches cruise speed, so these timings would be optimistic. "
                "Lower cruise_speed_cm_s or raise accel_cm_s2."
            )

    # -- timing --------------------------------------------------------------

    def cruise_ms(self, distance_cm: int) -> int:
        """Time to cover `distance_cm` at steady speed."""
        return math.ceil(1000.0 * distance_cm / self.cruise_speed_cm_s)

    def stop_and_go_ms(self) -> int:
        """Extra time a full stop and restart costs over cruising through."""
        return math.ceil(1000.0 * self.cruise_speed_cm_s / self.accel_cm_s2)

    def half_stop_ms(self) -> int:
        """Extra time for only one half of that: starting from rest, or stopping
        for good at the goal."""
        return math.ceil(500.0 * self.cruise_speed_cm_s / self.accel_cm_s2)

    def turn_ms(self, quarter_turns: int) -> int:
        """Rotation time. `quarter_turns` is 0, 1, or 2 for a bay reversal."""
        return quarter_turns * self.turn_ms_per_quarter

    # -- energy --------------------------------------------------------------

    def cruise_energy_j(self, distance_cm: int) -> float:
        return self.rolling_j_per_cm * distance_cm

    def stop_and_go_energy_j(self) -> float:
        return self.stop_and_go_j

    def turn_energy_j(self, quarter_turns: int) -> float:
        return self.turn_j_per_quarter * quarter_turns

    def idle_energy_j(self, duration_ms: int) -> float:
        return self.idle_w * duration_ms / 1000.0


DEFAULT_KINEMATICS = RobotKinematics()
