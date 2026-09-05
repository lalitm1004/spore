"""What a manoeuvre is worth, combining time and energy.

A battery-bound AMR does not simply want the fastest route. A path that is quicker
in wall-clock but pays for it in turns and stop-starts can be the worse choice when
the pack is low, and the planner has to be able to express that trade-off rather
than pick one objective and ignore the other.

So every manoeuvre is priced as

    cost = w_time * milliseconds + w_energy * joules

and the weights come from the robot's own energy state, which arrives in the
heartbeat. At `OK` the clock dominates and energy is a mild tie-breaker; by
`CRITICAL` energy dominates outright. `urgent` scales the time weight back up, so a
priority job can still buy speed with charge.

The practical consequence is the wait-versus-detour decision, which the search makes
for free once waiting carries its idle draw. With the default constants, one extra
hop of detour costs 2000 ms and 400 J while a five-second wait costs 5000 ms and
150 J -- so an `OK` robot detours and a `CRITICAL` robot waits, which is what you
want in both cases.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from spore_planner.planner.kinematics import DEFAULT_KINEMATICS, Kinematics


class EnergyState(StrEnum):
    """The robot's own assessment of its charge, as carried in the heartbeat."""

    OK = "OK"
    RECOVERING = "RECOVERING"
    SHORT = "SHORT"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True, slots=True)
class CostWeights:
    """Price of a millisecond and of a joule, in the same arbitrary unit."""

    time: float
    energy: float

    def __post_init__(self) -> None:
        if self.time <= 0 or self.energy < 0:
            raise ValueError("time weight must be positive and energy weight non-negative")


WEIGHTS_BY_ENERGY_STATE: dict[EnergyState, CostWeights] = {
    EnergyState.OK: CostWeights(time=1.0, energy=1.5),
    EnergyState.RECOVERING: CostWeights(time=1.0, energy=4.0),
    EnergyState.SHORT: CostWeights(time=1.0, energy=10.0),
    EnergyState.CRITICAL: CostWeights(time=1.0, energy=30.0),
}

URGENT_TIME_BOOST = 2.5
"""How much `urgent` scales the time weight. Bounded, so an urgent CRITICAL robot
still respects its battery rather than sprinting itself flat."""


@dataclass(frozen=True, slots=True)
class HopCost:
    """Duration, energy and blended cost of one manoeuvre."""

    duration_ms: int
    energy_j: float
    cost: float

    def __add__(self, other: HopCost) -> HopCost:
        return HopCost(
            duration_ms=self.duration_ms + other.duration_ms,
            energy_j=self.energy_j + other.energy_j,
            cost=self.cost + other.cost,
        )


ZERO_COST = HopCost(duration_ms=0, energy_j=0.0, cost=0.0)


@dataclass(frozen=True, slots=True)
class CostModel:
    """Prices manoeuvres for one robot in one energy state."""

    kinematics: Kinematics
    weights: CostWeights
    node_spacing_cm: int

    @classmethod
    def for_state(
        cls,
        node_spacing_cm: int,
        energy_state: EnergyState = EnergyState.OK,
        *,
        urgent: bool = False,
        kinematics: Kinematics | None = None,
    ) -> CostModel:
        kinematics = kinematics if kinematics is not None else DEFAULT_KINEMATICS
        base = WEIGHTS_BY_ENERGY_STATE[energy_state]
        weights = CostWeights(
            time=base.time * (URGENT_TIME_BOOST if urgent else 1.0),
            energy=base.energy,
        )
        return cls(kinematics=kinematics, weights=weights, node_spacing_cm=node_spacing_cm)

    def _blend(self, duration_ms: int, energy_j: float) -> HopCost:
        return HopCost(
            duration_ms=duration_ms,
            energy_j=energy_j,
            cost=self.weights.time * duration_ms + self.weights.energy * energy_j,
        )

    def hop(self, *, quarter_turns: int = 0, wait_ms: int = 0, moving: bool = True) -> HopCost:
        """Cost of turning, waiting, then crossing one edge.

        `moving` says whether the robot arrived at this node with speed on. It is
        false only at the very start of a path, for a robot standing still. A robot
        that turns or waits has stopped, and it stops once however many reasons it
        has to -- turning and waiting at the same node pays one stop, not two.
        """
        if quarter_turns < 0 or wait_ms < 0:
            raise ValueError("quarter_turns and wait_ms must be non-negative")
        kin = self.kinematics
        stops_here = quarter_turns > 0 or wait_ms > 0

        if not moving:
            # Already stopped: only the acceleration half is owed.
            penalty_ms = kin.half_stop_ms()
            penalty_j = kin.stop_and_go_energy_j() / 2.0
        elif stops_here:
            penalty_ms = kin.stop_and_go_ms()
            penalty_j = kin.stop_and_go_energy_j()
        else:
            penalty_ms = 0
            penalty_j = 0.0

        duration = (
            penalty_ms
            + kin.turn_ms(quarter_turns)
            + wait_ms
            + kin.cruise_ms(self.node_spacing_cm)
        )
        energy = (
            penalty_j
            + kin.turn_energy_j(quarter_turns)
            + kin.idle_energy_j(wait_ms)
            + kin.cruise_energy_j(self.node_spacing_cm)
        )
        return self._blend(duration, energy)

    def arrive(self) -> HopCost:
        """Cost of the final deceleration when the robot stops at its goal."""
        kin = self.kinematics
        return self._blend(kin.half_stop_ms(), kin.stop_and_go_energy_j() / 2.0)

    def wait_in_place(self, duration_ms: int) -> HopCost:
        """Cost of standing still for `duration_ms` without moving on."""
        if duration_ms < 0:
            raise ValueError("duration_ms must be non-negative")
        return self._blend(duration_ms, self.kinematics.idle_energy_j(duration_ms))

    def min_hop_cost(self) -> float:
        """Cheapest any single hop can be: straight through, no turn, no wait.

        This is what makes the A* heuristic admissible -- multiplying the exact
        remaining hop count by this can never overestimate, because no hop has a way
        to cost less.
        """
        return self.hop(quarter_turns=0, wait_ms=0, moving=True).cost
