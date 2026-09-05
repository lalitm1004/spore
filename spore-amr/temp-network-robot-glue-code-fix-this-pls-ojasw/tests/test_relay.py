"""The relay routes commands to the right robot's stream."""

import queue

from temp_network_interface import NetworkToRobot, Relay


def test_delivers_to_attached_robot():
    relay = Relay()
    outbox = queue.Queue()
    relay.attach(7, outbox)

    command = NetworkToRobot(target_node_id=99, timestamp=1)
    assert relay.deliver(7, command) is True
    assert outbox.get_nowait() == command


def test_deliver_to_disconnected_robot_fails():
    relay = Relay()
    assert relay.deliver(7, NetworkToRobot(target_node_id=99, timestamp=1)) is False


def test_detach_stops_delivery():
    relay = Relay()
    relay.attach(7, queue.Queue())
    relay.detach(7)
    assert relay.deliver(7, NetworkToRobot(target_node_id=99, timestamp=1)) is False
