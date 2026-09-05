"""The warehouse map, as the network layer needs it.

Routing is what a network layer is for, so it holds the map. This is
deliberately its own small loader rather than an import from the Webots
project: the dependency runs the wrong way round otherwise -- the network layer
would depend on one particular robot implementation, when the whole point is
that any implementation speaking the schemas can connect.

It is also much less than the robot's graph. The robot needs lane bearings,
because it has to turn onto them; the network layer only needs to know **what
connects to what**, because it names nodes and never directions. `NetworkToRobot`
carries `target_node_id` and has no field for a turn.

Distances are in hops, never metres. Two nodes either side of a rack are close
in a straight line and a long drive apart, so a destination chosen on
straight-line distance is sometimes a trip round a corner.

Pure: no grpc, no I/O beyond reading the map once.
"""

from __future__ import annotations

import json
import pathlib
from collections import deque
from typing import Dict, List, Optional


class Graph:
    """Adjacency over the warehouse's nodes. Undirected: lanes run both ways."""

    def __init__(self, node_ids, edges):
        self.nodes: Dict[int, dict] = {int(n["id"]): n for n in node_ids}
        self._adjacency: Dict[int, List[int]] = {n: [] for n in self.nodes}
        for a, b in edges:
            if a in self._adjacency and b in self._adjacency:
                self._adjacency[a].append(b)
                self._adjacency[b].append(a)
        for node in self._adjacency:
            self._adjacency[node].sort()

    def neighbours(self, node_id: int) -> List[int]:
        return list(self._adjacency.get(node_id, ()))

    def _depths(self, start: int) -> Dict[int, int]:
        """Hop distance from `start` to everything it can reach.

        One breadth-first sweep answers it for every node at once, which is why
        picking a destination costs the same whether the fleet is one robot or
        a hundred.
        """
        depth = {start: 0}
        queue = deque([start])
        while queue:
            node = queue.popleft()
            for neighbour in self._adjacency[node]:
                if neighbour not in depth:
                    depth[neighbour] = depth[node] + 1
                    queue.append(neighbour)
        return depth

    def far_nodes(self, start: int, minimum_hops: int = 1) -> List[int]:
        """Every node at least `minimum_hops` lanes from `start`, sorted.

        Empty when nowhere is far enough -- a small map, or a robot on an
        island. The caller decides what to do about that; this will not quietly
        return somewhere near instead.
        """
        if start not in self._adjacency:
            return []
        depth = self._depths(start)
        return sorted(n for n, hops in depth.items()
                      if hops >= minimum_hops and n != start)

    def hops(self, start: int, goal: int) -> Optional[int]:
        """Lanes between two nodes, or None if there is no route."""
        if start not in self._adjacency:
            return None
        return self._depths(start).get(goal)


def load_map(path) -> Graph:
    """Build a graph from `warehouse.json`.

    Only ids and edges are read. Positions are the robot's concern: it turns a
    named node into a bearing, and it holds the same file to do it.
    """
    document = json.loads(pathlib.Path(path).read_text())
    edges = [(int(e["a"]), int(e["b"])) for e in document["edges"]]
    return Graph(document["nodes"], edges)
