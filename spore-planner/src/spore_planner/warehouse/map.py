"""Types mirroring `warehouse-map.schema.json`, plus the heading primitive.

Distances are centimetres and times are milliseconds throughout this package. The
map schema pins `units: "cm"` and makes every edge exactly `node_spacing` long and
axis-aligned, which is what restricts robots to 90 degree turns.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, StrEnum


class NodeType(StrEnum):
    """Node roles, as defined by the map and QR-code schemas."""

    PT = "PT"  # pass-through: a plain lane node, no-op beyond locating the robot
    TR = "TR"  # transfer: cargo pickup / dropoff
    CH = "CH"  # charging bay
    PK = "PK"  # parking bay
    YI = "YI"  # yield bay: a pull-over spur to let another robot pass


class Density(StrEnum):
    """How tightly travel lanes are packed in a region."""

    DENSE = "dense"
    MEDIUM = "medium"
    SPARSE = "sparse"


class Heading(IntEnum):
    """Axis-aligned heading, in the map's coordinate frame (+y is north).

    Values are contiguous and ordered anticlockwise so that the turn between two
    headings is a simple modular distance -- see `quarter_turns`.
    """

    E = 0
    N = 1
    W = 2
    S = 3

    @property
    def delta(self) -> tuple[int, int]:
        """Unit (dx, dy) step for this heading."""
        return _DELTAS[self]

    @property
    def opposite(self) -> Heading:
        return Heading((self + 2) % 4)


_DELTAS: dict[Heading, tuple[int, int]] = {
    Heading.E: (1, 0),
    Heading.N: (0, 1),
    Heading.W: (-1, 0),
    Heading.S: (0, -1),
}


def quarter_turns(a: Heading, b: Heading) -> int:
    """Number of 90 degree turns needed to rotate from `a` to `b`: 0, 1 or 2.

    A robot rotates in place, and turning left or right costs the same, so only the
    magnitude matters. 2 means a full reversal -- the manoeuvre every dead-end bay
    (every CH, PK and YI node on this map) forces on exit.
    """
    d = abs(int(a) - int(b))
    return min(d, 4 - d)


@dataclass(frozen=True, slots=True)
class Position:
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class Node:
    id: int
    name: str
    region_id: int
    node_type: NodeType
    position: Position


@dataclass(frozen=True, slots=True)
class Region:
    id: int
    name: str
    density: Density
    description: str


@dataclass(frozen=True, slots=True)
class Edge:
    """Undirected connection between two nodes, `length` cm centre to centre."""

    a: int
    b: int
    length: int


@dataclass(frozen=True, slots=True)
class Dimensions:
    width: int
    height: int


@dataclass(frozen=True, slots=True)
class WarehouseMap:
    schema_version: str
    units: str
    node_spacing: int
    dimensions: Dimensions
    regions: tuple[Region, ...]
    nodes: tuple[Node, ...]
    edges: tuple[Edge, ...]


def heading_between(a: Position, b: Position) -> Heading:
    """Heading to travel from `a` to `b`, which must be axis-aligned neighbours."""
    dx = b.x - a.x
    dy = b.y - a.y
    if dx and dy:
        raise ValueError(f"positions are not axis-aligned: {a} -> {b}")
    if dx > 0:
        return Heading.E
    if dx < 0:
        return Heading.W
    if dy > 0:
        return Heading.N
    if dy < 0:
        return Heading.S
    raise ValueError(f"positions coincide: {a}")
