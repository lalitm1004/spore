"""The companion's link to this robot's own network-layer bot.

One long-lived `RobotNetwork.Session` stream, and the only thing on the robot
side that knows there is a wire at all. Everything above it works in
`robot/network.py`'s `Query` and `Decision`.

WHY a stream, when a junction wants an answer now
    Because the robot says two different kinds of thing and only one of them is
    a question.

    It **reports** — where it is, how charged it is, what is in its way. That is
    the fleet's location interface, and it is why every bot used to think it
    stood at node 0 for ever: nothing carried this. Reports are fire and forget.

    It **asks** — I am at this node, these are the ways out, which one? The
    robot has stopped and is waiting, so this needs an answer, and `ask` is
    where the asynchronous stream is turned back into a bounded wait. Only here.

    The wait is bounded twice over: by `timeout_s`, and by the firmware's own
    junction timeout, which drives on regardless if nobody answers. A robot must
    never hold a lane because a service is slow.

WHY it degrades rather than raises
    No network layer, a timeout, a stream that died: all of them come back as
    `None`, and the caller decides what a robot does when nobody tells it where
    to go. This layer will not guess. Reconnection is lazy — the next call
    dials again — because a companion that gave up on the first failure would
    be a robot that never recovered from a bot restart.
"""

import queue
import threading

from robot.network import Decision, Query

#: Wire enum name -> our string. Imported lazily with the stubs below, so this
#: module can be imported without protobuf on the path (the webots tests do
#: exactly that) and only a real `connect()` needs it.
_KIND_NAMES = {
    0: "PROCEED",   # KIND_UNSPECIFIED: absent means take the lane
    1: "PROCEED",
    2: "REROUTE",
    3: "WAIT",
    4: "YIELD",
}


class Uplink:
    """Reports upward; asks when the robot is standing at a junction."""

    def __init__(self, address: str, timeout_s: float = 5.0,
                 bot_id: int = 0, battery_percent: float = 100.0):
        self.address = address
        self.timeout_s = timeout_s
        self.bot_id = bot_id
        self.battery_percent = battery_percent
        self._outbound: "queue.Queue" = queue.Queue()
        self._replies = None
        self._channel = None
        self._lock = threading.Lock()

    # ---- transport ---------------------------------------------------------

    def connect(self) -> bool:
        """Open the stream. False rather than an exception: a robot with no
        network layer is a situation, not a crash."""
        try:
            import grpc

            from proto import robot_pb2_grpc

            self._channel = grpc.insecure_channel(self.address)
            stub = robot_pb2_grpc.RobotNetworkStub(self._channel)
            self._replies = stub.Session(self._requests(), metadata=(
                ("bot-id", str(self.bot_id)), ("region-id", "0"), ("role", "robot"),
            ))
            return True
        except Exception:
            self.close()
            return False

    def _requests(self):
        """Feeds the outbound half of the stream.

        A queue rather than a generator over the caller, because the two halves
        run at different rates: reports go up whenever the firmware says
        something, and answers come back only for the ones that asked.
        """
        while True:
            message = self._outbound.get()
            if message is None:
                return
            yield message

    def close(self) -> None:
        self._outbound.put(None)
        if self._channel is not None:
            try:
                self._channel.close()
            finally:
                self._channel = None
        self._replies = None

    # ---- what the companion calls ------------------------------------------

    def report(self, node_id: int, region_id: int, *, battery: float = None,
               obstacle_node: int = None) -> None:
        """Tell the network layer where this robot is. No answer expected."""
        message = self._message(node_id, region_id, battery=battery,
                                obstacle_node=obstacle_node)
        if message is not None:
            self._outbound.put(message)

    def ask(self, query: Query):
        """Send a junction question and wait for its answer.

        Returns None on any failure — no network layer, a timeout, a stream that
        ended. Mismatched answers are discarded rather than returned: a late
        reply to the previous node would send this robot somewhere chosen for a
        place it has already left.
        """
        with self._lock:
            if self._replies is None and not self.connect():
                return None

            message = self._message(
                query.node_id, query.region_id,
                available=query.available, heading_rad=query.heading_rad,
                query_id=query.query_id)
            if message is None:
                return None
            self._outbound.put(message)

            try:
                for reply in self._replies:
                    if reply.query_id and reply.query_id != query.query_id:
                        continue  # a late answer to a junction already left
                    return Decision(
                        query_id=reply.query_id or query.query_id,
                        target_node_id=reply.target_node_id,
                        kind=_KIND_NAMES.get(reply.kind, "PROCEED"),
                        hold_ms=reply.hold_ms,
                        because=reply.because,
                    )
            except Exception:
                self.close()
            return None

    # ---- shaping -----------------------------------------------------------

    def _message(self, node_id: int, region_id: int, *, available=(),
                 heading_rad: float = 0.0, query_id: int = 0,
                 battery: float = None, obstacle_node: int = None):
        try:
            from proto import robot_pb2
        except ImportError:
            return None

        message = robot_pb2.RobotToNetwork(
            bot_id=self.bot_id,
            region_id=region_id,
            latest_node_id=node_id,
            heading_rad=heading_rad,
            query_id=query_id,
            available=list(available),
            mission=robot_pb2.Mission(idle=robot_pb2.Idle()),
            telemetry=robot_pb2.Telemetry(battery=robot_pb2.Battery(
                percentage=self.battery_percent if battery is None else battery)),
        )
        if obstacle_node:
            # The field that makes a reported blockage real. The shared schema
            # has always carried it; nothing ever sent it, so the planner only
            # ever heard about obstructions through an admin back door.
            message.fault.warning.obstacle.current_node_id = obstacle_node
        return message
