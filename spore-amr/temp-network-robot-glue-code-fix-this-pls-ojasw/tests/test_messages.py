"""The domain messages round-trip and reject invalid values at construction."""

import pytest

from temp_network_interface import (
    Battery,
    Cargo,
    Error,
    Fault,
    Mission,
    NetworkToRobot,
    RobotState,
    RobotToNetwork,
    Telemetry,
    Warning,
)


def status(**overrides):
    base = dict(
        bot_id=1,
        region_id=3,
        latest_node_id=10,
        mission=Mission(type="IDLE"),
        telemetry=Telemetry(battery=Battery(percentage=100.0)),
        timestamp=1_700_000_000,
    )
    base.update(overrides)
    return RobotToNetwork(**base)


def test_idle_status_round_trips_through_dict():
    message = status()
    assert RobotToNetwork.from_dict(message.to_dict()) == message


def test_cargo_mission_round_trips():
    message = status(
        mission=Mission(type="CARGO", cargo=Cargo(cargo_id="d290f1ee-6c54-4b01-90e6-d701748f0851",
                                                  state="EN_ROUTE")),
    )
    assert RobotToNetwork.from_dict(message.to_dict()) == message


def test_obstacle_fault_round_trips():
    message = status(fault=Fault(warning=Warning(type="OBSTACLE", current_node_id=10)))
    assert RobotToNetwork.from_dict(message.to_dict()) == message


def test_low_battery_fault_round_trips():
    message = status(fault=Fault(warning=Warning(type="LOW_BATTERY", percentage=12.5)))
    assert RobotToNetwork.from_dict(message.to_dict()) == message


def test_error_fault_round_trips():
    message = status(fault=Fault(error=Error(type="MOTOR_ERROR")))
    assert RobotToNetwork.from_dict(message.to_dict()) == message


def test_command_with_mission_round_trips():
    message = NetworkToRobot(
        target_node_id=42, timestamp=1_700_000_001, set_mission=Mission(type="PARK")
    )
    assert NetworkToRobot.from_dict(message.to_dict()) == message


def test_command_without_mission_round_trips():
    message = NetworkToRobot(target_node_id=42, timestamp=1_700_000_001)
    assert NetworkToRobot.from_dict(message.to_dict()) == message


def test_cargo_mission_requires_cargo():
    with pytest.raises(ValueError):
        Mission(type="CARGO")


def test_unknown_mission_type_is_rejected():
    with pytest.raises(ValueError):
        Mission(type="EXPLODE")


def test_unknown_cargo_state_is_rejected():
    with pytest.raises(ValueError):
        Cargo(cargo_id="d290f1ee-6c54-4b01-90e6-d701748f0851", state="FLYING")


def test_unknown_error_type_is_rejected():
    with pytest.raises(ValueError):
        Error(type="PILOT_ERROR")


def test_robot_state_projects_a_command():
    command = NetworkToRobot(target_node_id=42, timestamp=9, set_mission=Mission(type="PARK"))
    state = RobotState.from_command(command)
    assert state.target_node_id == 42
    assert state.mission == Mission(type="PARK")
    assert state.timestamp == 9


def test_robot_state_defaults_to_no_goal():
    state = RobotState()
    assert state.target_node_id is None
    assert state.mission is None
    assert state.timestamp is None
