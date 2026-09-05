"""End-to-end: a live gRPC server and real clients, over loopback."""

from concurrent import futures

import grpc

from temp_network_interface import (
    HoldPolicy,
    NetworkToRobot,
    NoopPolicy,
    Relay,
    TargetedCommand,
    network_pb2_grpc,
)
from temp_network_interface.client import NetworkClient
from temp_network_interface.server import NetworkService
from temp_network_interface.state import Fleet

from .test_messages import status


class _Server:
    def __init__(self, fleet=None, policy=None):
        self._server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
        network_pb2_grpc.add_RobotNetworkServicer_to_server(
            NetworkService(fleet=fleet, policy=policy), self._server)
        self.port = self._server.add_insecure_port("127.0.0.1:0")
        self._server.start()

    def stop(self):
        self._server.stop(grace=0.5)


def _client(server):
    return NetworkClient("127.0.0.1:{}".format(server.port))


def test_status_up_command_back():
    server = _Server(policy=HoldPolicy())
    try:
        with _client(server) as client:
            assert client.send(status(bot_id=1, latest_node_id=77))
            command = client.recv(timeout=5.0)
            # The robot's state is preserved as its current goal, not just
            # surfaced once through recv.
            assert client.state.target_node_id == 77
            assert client.state.mission.type == "HOLD"
    finally:
        server.stop()

    assert command is not None
    assert command.target_node_id == 77
    assert command.set_mission.type == "HOLD"


def test_noop_policy_returns_no_command():
    server = _Server(policy=NoopPolicy())
    try:
        with _client(server) as client:
            client.send(status())
            assert client.recv(timeout=0.3) is None
    finally:
        server.stop()


def test_fleet_tracks_every_robot():
    fleet = Fleet()
    server = _Server(fleet=fleet, policy=HoldPolicy())
    try:
        with _client(server) as client:
            client.send(status(bot_id=2, latest_node_id=5))
            client.recv(timeout=5.0)
    finally:
        server.stop()

    assert len(fleet) == 1
    assert fleet.robot(2).latest_node_id == 5


class _RedirectPolicy:
    """On bot 1's message, ack bot 1 and command bot 2. The ack lets a test
    wait until bot 1's message has actually been processed, removing the race
    between `send` and `close`."""

    def on_status(self, fleet, status):
        if status.bot_id == 1:
            return [
                TargetedCommand(bot_id=1, command=NetworkToRobot(
                    target_node_id=111, timestamp=status.timestamp)),
                TargetedCommand(bot_id=2, command=NetworkToRobot(
                    target_node_id=999, timestamp=status.timestamp)),
            ]
        return []


def test_command_can_target_a_different_robot():
    server = _Server(policy=_RedirectPolicy())
    try:
        with _client(server) as c1, _client(server) as c2:
            c2.send(status(bot_id=2, latest_node_id=50))  # attach bot 2
            c1.send(status(bot_id=1, latest_node_id=10))  # triggers a bot-2 command
            command = c2.recv(timeout=5.0)
    finally:
        server.stop()

    assert command is not None
    assert command.target_node_id == 999


def test_pending_command_delivered_on_reconnect():
    server = _Server(policy=_RedirectPolicy())
    try:
        # Bot 2 connects and then goes away before it is commanded.
        with _client(server) as c2:
            c2.send(status(bot_id=2, latest_node_id=50))

        # Bot 1 triggers a command for the now-disconnected bot 2; the ack to
        # bot 1 is the signal that the command has been recorded.
        with _client(server) as c1:
            c1.send(status(bot_id=1, latest_node_id=10))
            ack = c1.recv(timeout=5.0)
            assert ack is not None and ack.target_node_id == 111

        # Bot 2 comes back and should receive the pending command.
        with _client(server) as c2b:
            c2b.send(status(bot_id=2, latest_node_id=50))
            command = c2b.recv(timeout=5.0)
    finally:
        server.stop()

    assert command is not None
    assert command.target_node_id == 999
