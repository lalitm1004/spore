"""World-state reconciliation: commands stay outstanding until fulfilled."""

from temp_network_interface import Cargo, Fleet, Mission, NetworkToRobot
from temp_network_interface.state import fulfilled

from .test_messages import status

CARGO_ID = "d290f1ee-6c54-4b01-90e6-d701748f0851"


def nav_command(node):
    return NetworkToRobot(target_node_id=node, timestamp=1)


def mission_command(mission):
    return NetworkToRobot(target_node_id=1, set_mission=mission, timestamp=1)


def test_navigation_command_fulfilled_on_arrival():
    command = nav_command(99)
    assert fulfilled(command, status(latest_node_id=99))
    assert not fulfilled(command, status(latest_node_id=10))


def test_mission_command_fulfilled_on_mission_match():
    command = mission_command(Mission(type="HOLD"))
    assert fulfilled(command, status(mission=Mission(type="HOLD")))
    assert not fulfilled(command, status(mission=Mission(type="IDLE")))


def test_cargo_command_fulfilled_only_on_cargo_match():
    cargo = Cargo(cargo_id=CARGO_ID, state="PICKUP")
    command = mission_command(Mission(type="CARGO", cargo=cargo))

    assert fulfilled(command, status(mission=Mission(type="CARGO", cargo=cargo)))
    assert not fulfilled(command, status(mission=Mission(
        type="CARGO", cargo=Cargo(cargo_id=CARGO_ID, state="EN_ROUTE"))))
    assert not fulfilled(command, status(mission=Mission(
        type="CARGO", cargo=Cargo(cargo_id="some-other-id", state="PICKUP"))))


def test_update_reconciles_fulfilled_commands():
    fleet = Fleet()
    fleet.queue(5, nav_command(99))
    fleet.update(status(bot_id=5, latest_node_id=99))
    assert fleet.pending(5) == []


def test_update_keeps_unfulfilled_commands():
    fleet = Fleet()
    fleet.queue(5, nav_command(99))
    fleet.update(status(bot_id=5, latest_node_id=10))
    assert len(fleet.pending(5)) == 1


def test_record_without_journal_is_in_memory_only():
    fleet = Fleet()
    fleet.record_status(status(bot_id=3))
    fleet.record_command(3, nav_command(99))
    assert fleet.robot(3).bot_id == 3
    assert len(fleet.pending(3)) == 1
