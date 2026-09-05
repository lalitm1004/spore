"""Timing and cost model.

The important properties are that timings never come out optimistic (they become
reservation windows, and a short window is unsafe), that a stop is charged once
however many reasons the robot has to make it, and that `min_hop_cost` really is a
floor -- the A* heuristic is only admissible because of it.
"""

from __future__ import annotations

import pytest

from spore_planner.planner.cost import (
    URGENT_TIME_BOOST,
    WEIGHTS_BY_ENERGY_STATE,
    CostModel,
    CostWeights,
    EnergyState,
    HopCost,
)
from spore_planner.planner.kinematics import DEFAULT_KINEMATICS, RobotKinematics

SPACING = 200


@pytest.fixture
def kin() -> RobotKinematics:
    return DEFAULT_KINEMATICS


# -- kinematics --------------------------------------------------------------


def test_cruise_time_is_linear_in_distance(kin):
    assert kin.cruise_ms(200) == 2000
    assert kin.cruise_ms(400) == 2 * kin.cruise_ms(200)


def test_a_full_stop_costs_about_twice_a_half_stop(kin):
    assert kin.stop_and_go_ms() == pytest.approx(2 * kin.half_stop_ms(), abs=1)


def test_turn_time_and_energy_are_linear_in_quarter_turns(kin):
    assert kin.turn_ms(0) == 0
    assert kin.turn_ms(2) == 2 * kin.turn_ms(1)
    assert kin.turn_energy_j(2) == 2 * kin.turn_energy_j(1)


def test_timings_round_up_so_reservations_are_never_too_short():
    # 3 cm at 7 cm/s is 428.57 ms; a floor would under-reserve.
    kin = RobotKinematics(cruise_speed_cm_s=7.0)
    assert kin.cruise_ms(3) == 429


def test_idle_energy_is_proportional_to_time(kin):
    assert kin.idle_energy_j(1000) == kin.idle_w
    assert kin.idle_energy_j(0) == 0.0


def test_moving_draws_more_power_than_idling(kin):
    # The relationship the wait-versus-detour trade-off depends on.
    moving_w = kin.rolling_j_per_cm * kin.cruise_speed_cm_s
    assert moving_w > kin.idle_w


def test_validate_for_spacing_accepts_the_real_map(kin):
    kin.validate_for_spacing(SPACING)


def test_validate_for_spacing_rejects_edges_too_short_to_reach_cruise():
    kin = RobotKinematics(cruise_speed_cm_s=200.0, accel_cm_s2=50.0)
    with pytest.raises(ValueError, match="never reaches cruise speed"):
        kin.validate_for_spacing(SPACING)


def test_kinematics_rejects_nonsense_parameters():
    with pytest.raises(ValueError, match="cruise_speed_cm_s must be positive"):
        RobotKinematics(cruise_speed_cm_s=0.0)
    with pytest.raises(ValueError, match="accel_cm_s2 must be positive"):
        RobotKinematics(accel_cm_s2=-1.0)


# -- cost model --------------------------------------------------------------


@pytest.fixture
def model() -> CostModel:
    return CostModel.for_state(SPACING, EnergyState.OK)


def test_a_straight_pass_costs_only_the_cruise(model, kin):
    hop = model.hop()
    assert hop.duration_ms == kin.cruise_ms(SPACING)
    assert hop.energy_j == kin.cruise_energy_j(SPACING)


def test_turning_adds_a_stop_and_the_rotation(model, kin):
    straight, turned = model.hop(), model.hop(quarter_turns=1)
    assert turned.duration_ms - straight.duration_ms == kin.stop_and_go_ms() + kin.turn_ms(1)
    assert turned.energy_j - straight.energy_j == pytest.approx(
        kin.stop_and_go_energy_j() + kin.turn_energy_j(1)
    )


def test_a_robot_that_turns_and_waits_pays_for_one_stop_not_two(model, kin):
    both = model.hop(quarter_turns=1, wait_ms=3000)
    expected = (
        kin.stop_and_go_ms() + kin.turn_ms(1) + 3000 + kin.cruise_ms(SPACING)
    )
    assert both.duration_ms == expected


def test_starting_from_rest_pays_only_the_acceleration_half(model, kin):
    from_rest = model.hop(moving=False)
    assert from_rest.duration_ms == kin.half_stop_ms() + kin.cruise_ms(SPACING)
    # And a robot already stopped does not pay a second stop to turn.
    turning_from_rest = model.hop(quarter_turns=1, moving=False)
    assert turning_from_rest.duration_ms == (
        kin.half_stop_ms() + kin.turn_ms(1) + kin.cruise_ms(SPACING)
    )


def test_the_bay_reversal_is_a_full_180(model, kin):
    # Every CH, PK and YI node on the real map is a dead end, so this is the common
    # case for charge and park missions, not an exotic one.
    assert model.hop(quarter_turns=2).duration_ms - model.hop().duration_ms == (
        kin.stop_and_go_ms() + kin.turn_ms(2)
    )


def test_arriving_pays_a_final_deceleration(model, kin):
    assert model.arrive().duration_ms == kin.half_stop_ms()


def test_wait_in_place_costs_only_idle_draw(model, kin):
    wait = model.wait_in_place(4000)
    assert wait.duration_ms == 4000
    assert wait.energy_j == kin.idle_energy_j(4000)


def test_hop_rejects_negative_inputs(model):
    with pytest.raises(ValueError, match="must be non-negative"):
        model.hop(quarter_turns=-1)
    with pytest.raises(ValueError, match="must be non-negative"):
        model.hop(wait_ms=-1)
    with pytest.raises(ValueError, match="must be non-negative"):
        model.wait_in_place(-1)


@pytest.mark.parametrize("energy_state", list(EnergyState))
@pytest.mark.parametrize("quarter_turns", [0, 1, 2])
@pytest.mark.parametrize("wait_ms", [0, 500, 9000])
@pytest.mark.parametrize("moving", [True, False])
def test_min_hop_cost_is_a_genuine_floor(energy_state, quarter_turns, wait_ms, moving):
    model = CostModel.for_state(SPACING, energy_state)
    hop = model.hop(quarter_turns=quarter_turns, wait_ms=wait_ms, moving=moving)
    assert hop.cost >= model.min_hop_cost()


def test_energy_weight_rises_as_the_battery_falls():
    weights = [WEIGHTS_BY_ENERGY_STATE[s].energy for s in EnergyState]
    assert weights == sorted(weights)


def test_urgent_boosts_the_time_weight_only():
    plain = CostModel.for_state(SPACING, EnergyState.CRITICAL)
    urgent = CostModel.for_state(SPACING, EnergyState.CRITICAL, urgent=True)
    assert urgent.weights.time == plain.weights.time * URGENT_TIME_BOOST
    assert urgent.weights.energy == plain.weights.energy


def test_a_critical_robot_prefers_waiting_where_an_ok_robot_detours():
    # The trade-off the energy term exists to express: is a five second wait worth
    # more or less than one extra hop of detour?
    wait_ms = 5000
    for state, expect_wait in [(EnergyState.OK, False), (EnergyState.CRITICAL, True)]:
        model = CostModel.for_state(SPACING, state)
        waiting = model.hop(wait_ms=wait_ms).cost
        detouring = model.hop().cost + model.hop(quarter_turns=1).cost
        assert (waiting < detouring) is expect_wait, state


def test_hop_costs_add():
    a = HopCost(duration_ms=10, energy_j=1.0, cost=100.0)
    b = HopCost(duration_ms=5, energy_j=2.0, cost=50.0)
    assert a + b == HopCost(duration_ms=15, energy_j=3.0, cost=150.0)


def test_cost_weights_are_validated():
    with pytest.raises(ValueError, match="time weight must be positive"):
        CostWeights(time=0.0, energy=1.0)
    with pytest.raises(ValueError, match="non-negative"):
        CostWeights(time=1.0, energy=-1.0)
