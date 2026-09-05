"""The companion's half of a junction: map lookup, then ask the network layer.

The firmware reports arriving at a node and waits. This turns that into a
question the network layer can answer, and the answer into a heading the
firmware can turn to.

The map lives here, not in the firmware. The firmware is the MCU: it drives,
and it must not acquire a dependency on a map file or a network socket. The
companion is the Pi, and holding `warehouse.json` is exactly its job.

Loading and socket I/O are separated from the decision, so the decision is
testable without either.
"""

import json
import math
import pathlib
import socket
from typing import Dict, Optional

from robot.network import Decision, Query
from tools.track.graph import Edge, Graph, Node


def load_map(path: pathlib.Path) -> Graph:
    """Build a graph from `warehouse.json`.

    Positions are centimetres from the plane's corner in the file, and metres
    in the world frame here, so this is also where that conversion lives --
    once, rather than scattered through the callers.
    """
    document = json.loads(pathlib.Path(path).read_text())
    width_cm = float(document["dimensions"]["width"])
    height_cm = float(document["dimensions"]["height"])

    nodes = [
        Node(
            node_id=int(n["id"]),
            x=float(n["position"]["x"]) / 100.0 - width_cm / 200.0,
            y=float(n["position"]["y"]) / 100.0 - height_cm / 200.0,
            kind=n.get("node_type", "PT"),
            name=n.get("name", ""),
            region_id=int(n.get("region_id", 0)),
        )
        for n in document["nodes"]
    ]
    edges = [Edge(int(e["a"]), int(e["b"])) for e in document["edges"]]
    return Graph(nodes, edges)


class Navigator:
    """Turns a marker arrival into a heading to turn to."""

    def __init__(self, graph: Graph, turn_tolerance_deg: float = 45.0):
        self.graph = graph
        self.turn_tolerance_deg = turn_tolerance_deg
        self.query_id = 0
        self.last_node: Optional[int] = None
        self.dead_ends = 0

    def heading_into(self, node_id: int) -> Optional[float]:
        """The heading a robot must have had to arrive here from the last node.

        Preferred over the robot's own odometry heading because it is exact:
        lanes are straight, so the bearing between two nodes *is* the direction
        of travel. Odometry drift never enters the turn calculation.
        """
        if self.last_node is None or self.last_node == node_id:
            return None
        if node_id not in self.graph.neighbours(self.last_node):
            # The robot did not arrive along a lane we know about. Trusting a
            # bearing across a gap would silently produce a wrong turn.
            return None
        return self.graph.bearing(self.last_node, node_id)

    def build_query(self, node_id: int, heading_rad: float) -> Query:
        """What to ask the network layer, with the legal turns resolved."""
        if node_id not in self.graph.nodes:
            raise KeyError("node {} is not in the map".format(node_id))

        exact = self.heading_into(node_id)
        heading = exact if exact is not None else heading_rad
        available: Dict[str, int] = self.graph.turns_from(
            node_id, heading, tolerance_deg=self.turn_tolerance_deg)

        node = self.graph.nodes[node_id]
        self.query_id += 1
        return Query(
            query_id=self.query_id,
            node_id=node_id,
            node_type=node.kind,
            region_id=node.region_id,
            x_cm=round(node.x * 100, 1),
            y_cm=round(node.y * 100, 1),
            heading_rad=heading,
            available=available,
        )

    def bearing_for(self, node_id: int, decision: Decision) -> Optional[float]:
        """Absolute heading the firmware should turn to, in radians."""
        if decision.target_node_id not in self.graph.neighbours(node_id):
            return None
        return self.graph.bearing(node_id, decision.target_node_id)

    def arrived(self, node_id: int) -> None:
        self.last_node = node_id


class NetworkLink:
    """A newline-delimited JSON connection to this robot's network layer.

    Deliberately blocking with a timeout rather than async: the companion has
    nothing else to do while a robot waits at a junction, and a timeout that
    fires is more useful than a callback that never does.
    """

    def __init__(self, path: pathlib.Path, timeout_s: float = 5.0):
        self.path = pathlib.Path(path)
        self.timeout_s = timeout_s
        self._socket: Optional[socket.socket] = None
        self._buffer = b""

    def connect(self) -> bool:
        try:
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.settimeout(self.timeout_s)
            connection.connect(str(self.path))
        except OSError:
            return False
        self._socket = connection
        self._buffer = b""
        return True

    def ask(self, query: Query) -> Optional[Decision]:
        """Send a query and wait for its answer.

        Returns None on any failure -- no network layer, a timeout, a dead end
        it declined to answer. The caller decides what a robot does when
        nobody tells it where to go; this layer will not guess.
        """
        if self._socket is None and not self.connect():
            return None

        try:
            self._socket.sendall((query.to_json() + "\n").encode("utf-8"))
            while b"\n" not in self._buffer:
                chunk = self._socket.recv(4096)
                if not chunk:
                    self.close()
                    return None
                self._buffer += chunk
        except OSError:
            self.close()
            return None

        line, _, self._buffer = self._buffer.partition(b"\n")
        try:
            decision = Decision.from_json(line.decode("utf-8", "replace"))
        except (ValueError, KeyError):
            return None

        if decision.query_id != query.query_id:
            # A late answer to a previous junction. Two junctions can share a
            # target, so the id is the only way to tell them apart.
            return None
        return decision

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
        self._buffer = b""


def degrees(radians: float) -> float:
    return math.degrees(radians)
