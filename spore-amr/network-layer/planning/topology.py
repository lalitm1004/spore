"""Corridor structure of the navigable graph.

This map is a corridor network rather than an open grid: 881 nodes carry only 952
edges, so the average degree is 2.16 and 609 nodes have degree exactly 2. Inside a
run of degree-2 nodes there is nowhere to pass, which means "route around the
traffic" is only ever a decision taken at a junction.

`Topology` precomputes that structure once per map:

* **bays** -- degree-1 dead ends. Every CH, PK and YI node on this map is one, so
  every charge, park and yield manoeuvre ends in a 180 degree reversal.
* **junctions** -- degree >= 3, the only places two robots can pass each other.
* **corridors** -- the maximal degree-2 chains between them.

The planner uses this for the soft congestion field and for the yield cascade. It
deliberately does *not* use it for corridor admission control: preventing two
robots from entering opposite ends of the same corridor is conflict resolution and
belongs to the layer that owns the priority ordering. What the planner does do is
report the corridor it is entering, so that layer has the information it needs.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

from planning.graph import Graph


@dataclass(frozen=True, slots=True)
class Corridor:
    """A maximal run of degree-2 nodes, bounded by two non-degree-2 endpoints.

    `nodes` runs endpoint to endpoint inclusive, so a corridor with no interior is
    just two adjacent endpoints. For a pure cycle the first and last entries are the
    same node.
    """

    nodes: tuple[int, ...]

    @property
    def hops(self) -> int:
        return len(self.nodes) - 1

    @property
    def interior(self) -> tuple[int, ...]:
        """The degree-2 nodes strictly between the endpoints."""
        return self.nodes[1:-1]

    @property
    def is_cycle(self) -> bool:
        return len(self.nodes) > 1 and self.nodes[0] == self.nodes[-1]

    def endpoints(self) -> tuple[int, int]:
        return self.nodes[0], self.nodes[-1]


class Topology:
    """Degree classification and corridor decomposition for one `Graph`."""

    __slots__ = ("_corridor_of_step", "bays", "corridors", "graph", "junctions")

    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        self.bays = frozenset(i for i in range(graph.n) if graph.degree(i) == 1)
        self.junctions = frozenset(i for i in range(graph.n) if graph.degree(i) >= 3)
        self.corridors = tuple(_decompose(graph))

        # Directed step -> the corridor that step travels along. Every interior edge
        # of a corridor maps to it in both directions.
        step_of: dict[tuple[int, int], int] = {}
        for index, corridor in enumerate(self.corridors):
            for u, v in pairwise(corridor.nodes):
                step_of[(u, v)] = index
                step_of[(v, u)] = index
        self._corridor_of_step = step_of

    def corridor_entered_by(self, u: int, v: int) -> Corridor | None:
        """The corridor a robot commits to by stepping from `u` to `v`, if any."""
        index = self._corridor_of_step.get((u, v))
        return None if index is None else self.corridors[index]

    def is_junction(self, i: int) -> bool:
        return i in self.junctions

    def is_bay(self, i: int) -> bool:
        return i in self.bays

    def degree_histogram(self) -> dict[int, int]:
        """Node count by degree -- the shape check the map regression test asserts."""
        histogram: dict[int, int] = {}
        for i in range(self.graph.n):
            degree = self.graph.degree(i)
            histogram[degree] = histogram.get(degree, 0) + 1
        return dict(sorted(histogram.items()))

    def longest_corridor(self) -> Corridor | None:
        """The corridor with the most hops -- the worst case for head-on deadlock."""
        if not self.corridors:
            return None
        return max(self.corridors, key=lambda c: (c.hops, c.nodes))

    def __repr__(self) -> str:
        return (
            f"<Topology {len(self.corridors)} corridors, "
            f"{len(self.junctions)} junctions, {len(self.bays)} bays>"
        )


def _decompose(graph: Graph) -> list[Corridor]:
    """Split the graph into maximal degree-2 chains between non-degree-2 endpoints."""
    corridors: list[Corridor] = []
    walked: set[tuple[int, int]] = set()

    endpoints = [i for i in range(graph.n) if graph.degree(i) != 2]
    for start in endpoints:
        for first, _ in graph.neighbours(start):
            if (start, first) in walked:
                continue
            chain = _walk(graph, start, first, walked)
            corridors.append(Corridor(nodes=tuple(chain)))

    # Anything still unwalked is a ring of degree-2 nodes with no endpoint to start
    # from. This map has none, but a generator change could introduce one and a
    # silently dropped ring would be a nasty way to find out.
    for i in range(graph.n):
        if graph.degree(i) != 2:
            continue
        for first, _ in graph.neighbours(i):
            if (i, first) in walked:
                continue
            chain = _walk(graph, i, first, walked)
            corridors.append(Corridor(nodes=tuple(chain)))

    return corridors


def _walk(graph: Graph, start: int, first: int, walked: set[tuple[int, int]]) -> list[int]:
    """Follow degree-2 nodes from `start` via `first` until the chain ends."""
    chain = [start, first]
    walked.add((start, first))
    walked.add((first, start))
    previous, current = start, first
    while graph.degree(current) == 2 and current != start:
        nxt = next(v for v, _ in graph.neighbours(current) if v != previous)
        walked.add((current, nxt))
        walked.add((nxt, current))
        chain.append(nxt)
        previous, current = current, nxt
    return chain
