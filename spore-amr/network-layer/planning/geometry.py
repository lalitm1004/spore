"""The primitives the planner reasons with: node kinds, and which way is which.

WHAT
    `NodeType` and `Density` as the map schema defines them, `Position`,
    `Heading` for the four axis-aligned directions, and the two helpers that
    turn geometry into turns: `quarter_turns` (how far to rotate) and
    `heading_between` (which way two adjacent nodes lie).

WHERE
    Imported across `planning/`. Deliberately holds no map data --
    `warehouse/map.py` loads the document once and this stays a vocabulary, so
    nothing here can become a second copy of the map.

WHY
    Heading belongs in the search state because robots rotate in place: what a
    hop costs depends on the direction it is entered from. Without it the search
    cannot tell a straight run from a zig-zag, and on a battery-bound robot that
    is exactly the difference worth seeing.

HOW
    `Heading` values are contiguous and ordered anticlockwise, so the turn
    between two of them is a modular distance rather than a lookup table, and a
    reversal is simply the maximum of it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum, StrEnum


class NodeType(StrEnum):
    """Node roles, as the map and QR-code schemas define them."""

    PT = "PT"  # pass-through: a plain lane node
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
    """Axis-aligned heading in the map frame, where +y is north."""

    E = 0
    N = 1
    W = 2
    S = 3

    @property
    def delta(self) -> tuple[int, int]:
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


@dataclass(frozen=True, slots=True)
class Position:
    x: float
    y: float


def quarter_turns(a: Heading, b: Heading) -> int:
    """Number of 90 degree turns to rotate from `a` to `b`: 0, 1 or 2.

    A robot rotates in place and turning left costs the same as right, so only
    the magnitude matters. 2 is a full reversal -- the manoeuvre every dead-end
    bay forces on exit, which on this map is every charging, parking and yield
    node.
    """
    d = abs(int(a) - int(b))
    return min(d, 4 - d)


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


def heading_from_radians(radians: float) -> Heading:
    """Nearest axis-aligned heading to a bearing.

    The robot sends the bearing it arrived on, which is exact — lanes are
    straight, so the angle between two nodes *is* the direction of travel. It
    still needs rounding to one of four, because that is all a lane can be.
    """
    quarter = round(radians / (math.pi / 2)) % 4
    return Heading(quarter)
