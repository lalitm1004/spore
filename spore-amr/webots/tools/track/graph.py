"""A lane graph: nodes joined by straight edges.

Replaces the single analytic `Oval` for tracks that branch. A robot only ever
has a choice where lanes meet, so junctions are the whole point -- without them
the five node types are five names for identical behaviour.

Edges are straight. That is not a simplification for the simulator's benefit:
`warehouse.json` lays its 881 nodes on a 2 m lattice with straight spans
between them, so a straight edge is the real geometry and the oval was the
outlier. It also means the bearing from one node to the next is exactly the
lane's direction, which is what lets a robot turn correctly at a junction
knowing only node positions.

Pure: no I/O, no Webots, no rendering.
"""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Which way a robot turns, relative to the heading it arrived on.
TURNS = ("left", "straight", "right")


@dataclass(frozen=True)
class Node:
    """One node, in metres in the world frame."""

    node_id: int
    x: float
    y: float
    kind: str = "PT"
    name: str = ""
    region_id: int = 0


@dataclass(frozen=True)
class Edge:
    """An undirected lane between two nodes. Robots may travel either way."""

    a: int
    b: int

    def other(self, node_id: int) -> int:
        if node_id == self.a:
            return self.b
        if node_id == self.b:
            return self.a
        raise ValueError("edge {}-{} does not touch node {}".format(
            self.a, self.b, node_id))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Graph:
    """Nodes and the straight lanes between them."""

    def __init__(self, nodes: Sequence[Node], edges: Sequence[Edge]):
        self.nodes: Dict[int, Node] = {}
        for node in nodes:
            if node.node_id in self.nodes:
                raise ValueError("duplicate node id {}".format(node.node_id))
            self.nodes[node.node_id] = node

        self.edges: List[Edge] = []
        seen = set()
        for edge in edges:
            for endpoint in (edge.a, edge.b):
                if endpoint not in self.nodes:
                    raise ValueError("edge {}-{} names unknown node {}".format(
                        edge.a, edge.b, endpoint))
            if edge.a == edge.b:
                raise ValueError("edge {}-{} is a self-loop".format(edge.a, edge.b))
            key = (min(edge.a, edge.b), max(edge.a, edge.b))
            if key in seen:
                raise ValueError("duplicate edge {}-{}".format(edge.a, edge.b))
            seen.add(key)
            self.edges.append(edge)

        self._adjacency: Dict[int, List[int]] = {n: [] for n in self.nodes}
        for edge in self.edges:
            self._adjacency[edge.a].append(edge.b)
            self._adjacency[edge.b].append(edge.a)

    # ------------------------------------------------------------ topology --

    def neighbours(self, node_id: int) -> List[int]:
        return list(self._adjacency[node_id])

    def degree(self, node_id: int) -> int:
        return len(self._adjacency[node_id])

    def bearing(self, from_node: int, to_node: int) -> float:
        """Direction of the lane from one node to another, radians from +x."""
        a, b = self.nodes[from_node], self.nodes[to_node]
        return math.atan2(b.y - a.y, b.x - a.x)

    def length(self, from_node: int, to_node: int) -> float:
        a, b = self.nodes[from_node], self.nodes[to_node]
        return math.hypot(b.x - a.x, b.y - a.y)

    @property
    def total_length(self) -> float:
        return sum(self.length(e.a, e.b) for e in self.edges)

    # ------------------------------------------------------------- turning --

    def turns_from(self, node_id: int, heading: float,
                   tolerance_deg: float = 45.0) -> Dict[str, int]:
        """Which of left/straight/right lead somewhere, and to which node.

        `heading` is how the robot arrived. The lane it came in on is excluded:
        a robot that has just driven into a node has no business calling the
        way it came a legal turn, and on a one-way lane it would be a wrong-way
        entry.

        Ambiguity is resolved by picking the closest lane to each ideal
        bearing, so a node with two lanes 30 degrees apart does not report both
        as "straight".
        """
        ideal = {"left": wrap_pi(heading + math.pi / 2),
                 "straight": wrap_pi(heading),
                 "right": wrap_pi(heading - math.pi / 2)}
        back = wrap_pi(heading + math.pi)
        tolerance = math.radians(tolerance_deg)

        candidates = []
        for neighbour in self._adjacency[node_id]:
            lane = self.bearing(node_id, neighbour)
            if abs(wrap_pi(lane - back)) < math.radians(1.0):
                continue  # the lane we arrived on
            candidates.append((neighbour, lane))

        if not candidates:
            # A dead end -- every charging bay in the real warehouse is one,
            # a degree-1 spur off a corridor. Excluding the arrival lane leaves
            # nothing, and a robot offered no turn sits in the bay for the rest
            # of the run. Reversing out is the only way, so offer it.
            candidates = [(n, self.bearing(node_id, n))
                          for n in self._adjacency[node_id]]

        result: Dict[str, int] = {}
        for turn, target in ideal.items():
            best, best_gap = None, tolerance
            for neighbour, lane in candidates:
                gap = abs(wrap_pi(lane - target))
                if gap < best_gap:
                    best, best_gap = neighbour, gap
            if best is not None:
                result[turn] = best

        # A lane can be the closest match for two turns; keep the better fit so
        # each neighbour is offered once.
        claimed: Dict[int, str] = {}
        for turn in TURNS:
            neighbour = result.get(turn)
            if neighbour is None:
                continue
            previous = claimed.get(neighbour)
            if previous is None:
                claimed[neighbour] = turn
                continue
            gap_now = abs(wrap_pi(self.bearing(node_id, neighbour) - ideal[turn]))
            gap_was = abs(wrap_pi(self.bearing(node_id, neighbour) - ideal[previous]))
            loser = turn if gap_now >= gap_was else previous
            if loser != turn:
                claimed[neighbour] = turn
            result.pop(loser, None)

        return result

    # -------------------------------------------------------- ground truth --

    def distance_to_lane(self, x: float, y: float) -> float:
        """Distance to the nearest lane, in metres.

        Unsigned, unlike the oval's cross-track error: a graph has no inside,
        so there is no side for a sign to name. Telemetry that wants a signed
        error has to say signed with respect to which edge.
        """
        return min(self._distance_to_edge(x, y, e) for e in self.edges)

    def _distance_to_edge(self, x: float, y: float, edge: Edge) -> float:
        a, b = self.nodes[edge.a], self.nodes[edge.b]
        dx, dy = b.x - a.x, b.y - a.y
        span = dx * dx + dy * dy
        if span <= 0:
            return math.hypot(x - a.x, y - a.y)
        t = max(0.0, min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / span))
        return math.hypot(x - (a.x + t * dx), y - (a.y + t * dy))

    def nearest_node(self, x: float, y: float) -> Node:
        return min(self.nodes.values(), key=lambda n: math.hypot(x - n.x, y - n.y))


# ------------------------------------------------------------- generators --

# Region ids by quadrant, so a marker's region_id means something rather than
# always being zero. Mirrors how warehouse.json partitions its floor.
def _quadrant(row: int, column: int, rows: int, columns: int) -> int:
    return 1 + (2 if row >= rows / 2 else 0) + (1 if column >= columns / 2 else 0)


def lattice(rows: int, columns: int, spacing: float,
            kinds: Optional[Dict[Tuple[int, int], str]] = None) -> Graph:
    """A rectangular lattice, centred on the origin.

    Node ids run row-major from 0, so `id // columns` is the row -- readable in
    a log without a lookup table.
    """
    if rows < 2 or columns < 2:
        raise ValueError("a lattice needs at least 2x2 nodes, got {}x{}".format(
            rows, columns))

    kinds = kinds or {}
    origin_x = -(columns - 1) * spacing / 2.0
    origin_y = -(rows - 1) * spacing / 2.0

    nodes = []
    for row in range(rows):
        for column in range(columns):
            node_id = row * columns + column
            kind = kinds.get((row, column), "PT")
            nodes.append(Node(
                node_id=node_id,
                x=origin_x + column * spacing,
                y=origin_y + row * spacing,
                kind=kind,
                name="{}/{}/{:03d}".format(_SLUGS.get(kind, "aisle"), kind, node_id),
                region_id=_quadrant(row, column, rows, columns),
            ))

    edges = []
    for row in range(rows):
        for column in range(columns):
            node_id = row * columns + column
            if column + 1 < columns:
                edges.append(Edge(node_id, node_id + 1))
            if row + 1 < rows:
                edges.append(Edge(node_id, node_id + columns))

    return Graph(nodes, edges)


_SLUGS = {"PT": "aisle", "TR": "transfer", "CH": "charging",
          "PK": "parking", "YI": "yield"}


def lane_points(graph: Graph) -> Iterable[Tuple[Tuple[float, float],
                                                Tuple[float, float]]]:
    """Every lane as a pair of endpoints, for the rasteriser."""
    for edge in graph.edges:
        a, b = graph.nodes[edge.a], graph.nodes[edge.b]
        yield (a.x, a.y), (b.x, b.y)
