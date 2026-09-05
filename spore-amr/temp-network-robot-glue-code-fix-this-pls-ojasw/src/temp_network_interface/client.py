"""Robot-side gRPC client: what the companion will use to reach the network.

One long-lived bidirectional stream. Status is pushed as `RobotToNetwork`
messages; commands arrive as `NetworkToRobot`. The robot's entire knowledge is
the latest command, preserved in `state` (its target node and mission) -- the
fleet is the network layer's concern, never the firmware's.

The two halves are decoupled by queues so the robot's control loop never blocks
on the network, and a stalled or absent network degrades to no-ops rather than
failure -- the same rule the firmware already applies to its serial link:
telemetry is expendable, timing is not.

Imports grpc lazily, so this module only pulls in the transport when it is
actually used.
"""

from __future__ import annotations

import queue
import threading
from typing import Optional

from temp_network_interface.messages import NetworkToRobot, RobotState, RobotToNetwork
from temp_network_interface.transport import decode, encode_robot_to_network

# A sentinel pushed onto the inbound queue when the stream ends, so `recv`
# returns None instead of blocking forever after the network goes away.
_CLOSED = object()

# Outbound messages that will not fit are dropped rather than queued. Status is
# a heartbeat; a newer one replaces it anyway.
_OUTBOUND_MAX = 64


class NetworkClient:
    def __init__(self, target: str):
        self.target = target
        self._channel = None
        self._stub = None
        self._responses = None
        self._reader = None
        self._outbound: "queue.Queue" = queue.Queue(maxsize=_OUTBOUND_MAX)
        self._inbound: "queue.Queue" = queue.Queue()
        self._closed = threading.Event()
        # The robot's current goal, as last told by the network. This is the
        # only state the robot holds; it is preserved across `recv` calls until
        # the next command replaces it.
        self.state = RobotState()

    # ------------------------------------------------------------ lifecycle --

    def connect(self) -> None:
        """Open the channel and start the stream. Safe to call before the
        network service is up: the call waits for readiness rather than failing,
        and `recv` simply times out until it is."""
        import grpc

        from temp_network_interface import network_pb2_grpc

        self._channel = grpc.insecure_channel(self.target)
        self._stub = network_pb2_grpc.RobotNetworkStub(self._channel)
        self._responses = self._stub.Session(self._requests(), wait_for_ready=True)
        self._reader = threading.Thread(target=self._drain, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._closed.set()
        if self._channel is not None:
            self._channel.close()
        if self._reader is not None:
            self._reader.join(timeout=5.0)

    def __enter__(self) -> "NetworkClient":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --------------------------------------------------------------- sending --

    def send(self, message: RobotToNetwork) -> bool:
        """Push a status report upward. Returns False if it could not be queued
        (disconnected, or the buffer is full), which the caller may ignore."""
        if self._stub is None or self._closed.is_set():
            return False
        try:
            self._outbound.put_nowait(encode_robot_to_network(message))
            return True
        except queue.Full:
            return False

    # ------------------------------------------------------------- receiving --

    def recv(self, timeout: Optional[float] = None) -> Optional[NetworkToRobot]:
        """Next command from the network, or None on timeout or stream end."""
        if self._stub is None:
            return None
        try:
            item = self._inbound.get(timeout=timeout)
        except queue.Empty:
            return None
        return None if item is _CLOSED else item

    # ------------------------------------------------------------ internals --

    def _requests(self):
        """The gRPC request side: pull from the outbound queue until closed."""
        while not self._closed.is_set():
            try:
                yield self._outbound.get(timeout=0.2)
            except queue.Empty:
                continue

    def _drain(self) -> None:
        """Consume the response stream on a thread, decoding into the inbound
        queue and updating `state`. A malformed payload is dropped, never fatal."""
        try:
            for envelope in self._responses:
                try:
                    message = decode(envelope)
                except Exception:
                    continue
                if isinstance(message, NetworkToRobot):
                    self.state = RobotState.from_command(message)
                    self._inbound.put(message)
        except Exception:
            # Closing the channel cancels the stream; that is the normal exit,
            # not an error the caller needs to see.
            pass
        finally:
            self._inbound.put(_CLOSED)
