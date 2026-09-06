"""The companion's half of a junction: the map, and what it is used for.

The firmware reports arriving at a node and waits. The map turns the node the
network layer names into a lane bearing the firmware can turn to.

The map lives here, not in the firmware. The firmware is the MCU: it drives,
and it must not acquire a dependency on a map file or a network socket. The
companion is the Pi, and holding `warehouse.json` is exactly its job.

Loading and socket I/O are separated from the decision, so the decision is
testable without either.
"""

import json
import pathlib
from typing import Optional

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

    def __init__(self, graph: Graph):
        self.graph = graph
        self.query_id = 0
        self.last_node: Optional[int] = None
        self.bad_answers = 0
        self.dead_ends = 0

    def build_query(self, node_id: int, heading_rad: float) -> Query:
        """What to ask the network layer, with the legal turns resolved.

        The robot resolves the turns, not the network layer: it holds the map
        and it knows the heading it actually arrived on, so what is physically
        possible is its call. `heading_into` is preferred over the reported
        heading because it is exact -- lanes are straight, so the bearing
        between the last two nodes *is* the direction of travel, with no
        odometry drift in it.
        """
        if node_id not in self.graph.nodes:
            raise KeyError("node {} is not in the map".format(node_id))

        exact = self.heading_into(node_id)
        heading = exact if exact is not None else heading_rad
        available = self.graph.exits_from(node_id, heading)

        node = self.graph.nodes[node_id]
        self.query_id += 1
        return Query(
            query_id=self.query_id,
            node_id=node_id,
            node_type=node.kind,
            region_id=node.region_id,
            heading_rad=heading,
            available=available,
        )

    def bearing_for(self, node_id: int, decision: Decision) -> Optional[float]:
        """Absolute heading the firmware should turn to, in radians."""
        if decision.target_node_id not in self.graph.neighbours(node_id):
            return None
        return self.graph.bearing(node_id, decision.target_node_id)

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

    def arrived(self, node_id: int) -> None:
        self.last_node = node_id


