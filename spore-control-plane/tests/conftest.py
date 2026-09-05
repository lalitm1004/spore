"""Shared test helpers: a mock ControlPlaneService gRPC server.

The network layer does not implement `controlplane.proto` yet, so tests spin
up a fake implementation here to exercise the control plane's own client
(dispatch) and the web layer end to end.
"""
from __future__ import annotations

import socket
from concurrent import futures

import grpc
import pytest

from spore_control_plane.proto import controlplane_pb2, controlplane_pb2_grpc


class MockControlPlane(controlplane_pb2_grpc.ControlPlaneServiceServicer):
    """Configurable in-memory implementation of ControlPlaneService."""

    def __init__(self) -> None:
        self.orders: list[controlplane_pb2.Order] = []
        self.ack = controlplane_pb2.DispatchAck(accepted=True, owner_region=14, note="ok")

    def DispatchOrder(self, request: controlplane_pb2.Order, context):
        self.orders.append(request)
        return self.ack


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def mock_server():
    mock = MockControlPlane()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    controlplane_pb2_grpc.add_ControlPlaneServiceServicer_to_server(mock, server)
    port = free_port()
    if server.add_insecure_port(f"127.0.0.1:{port}") == 0:
        raise RuntimeError(f"could not bind mock server to {port}")
    server.start()
    yield f"127.0.0.1:{port}", mock
    server.stop(0)
