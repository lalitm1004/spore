"""Alternative routes, kept as diffs rather than copies.

WHAT
    `RouteCache` — the route a robot is driving plus a few alternatives, and the
    rules for dropping one when it becomes impossible and promoting one when the
    primary does.

WHERE
    Held per job by `bot.Bot` and consulted by `planning.decide` when the route
    in hand stops working. Pure: no search, no traffic, no clock.

WHY
    Recomputing from scratch every time a lane closes is wasteful when the
    answer is usually "go round this one corridor", and on a fleet where a
    blocked aisle stalls several robots at once, they would all pay for it in
    the same tick.

    Storing four whole routes is the naive way to keep alternatives, and on a
    seventy-hop job that is mostly four copies of the same list. Alternatives
    diverge from the primary for a stretch and rejoin it, so each is kept as a
    **splice**: where it leaves, where it comes back, and the handful of nodes
    in between. On these bots memory is as real a budget as time.

HOW
    `splice_of` finds the divergence by walking in from both ends, so the stored
    diff is minimal. An alternative is dropped when a blocked node falls inside
    its replacement stretch — nodes it shares with the primary do not matter,
    because if those are blocked the primary is dead too and everything is
    replanned anyway.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Splice:
    """One alternative, as a diff against the primary route.

    `primary[:start] + nodes + primary[end:]` reconstructs it. `start` and `end`
    are indices into the primary; `nodes` is what runs between them.
    """

    start: int
    end: int
    nodes: tuple[int, ...]
    cost: float = 0.0

    @property
    def size(self) -> int:
        """Nodes actually stored — what the diff costs us."""
        return len(self.nodes)


def splice_of(primary: tuple[int, ...], alternate: tuple[int, ...], cost: float = 0.0) -> Splice | None:
    """The minimal diff turning `primary` into `alternate`, or None if identical.

    Walks in from both ends so a route that only detours in the middle stores
    only the middle.
    """
    if primary == alternate:
        return None
    if not primary or not alternate:
        return Splice(start=0, end=len(primary), nodes=tuple(alternate), cost=cost)

    head = 0
    limit = min(len(primary), len(alternate))
    while head < limit and primary[head] == alternate[head]:
        head += 1

    tail = 0
    while (
        tail < limit - head
        and primary[len(primary) - 1 - tail] == alternate[len(alternate) - 1 - tail]
    ):
        tail += 1

    return Splice(
        start=head,
        end=len(primary) - tail,
        nodes=tuple(alternate[head : len(alternate) - tail]),
        cost=cost,
    )


@dataclass(frozen=True, slots=True)
class RouteCache:
    """A primary route and up to `ROUTE_ALTERNATES` diffs against it."""

    primary: tuple[int, ...]
    alternates: tuple[Splice, ...] = ()

    @classmethod
    def build(
        cls,
        primary: tuple[int, ...],
        alternates: tuple[tuple[int, ...], ...] = (),
        costs: tuple[float, ...] = (),
        limit: int = 3,
    ) -> RouteCache:
        spliced: list[Splice] = []
        for i, alternate in enumerate(alternates):
            if len(spliced) >= limit:
                break
            diff = splice_of(primary, alternate, cost=costs[i] if i < len(costs) else 0.0)
            if diff is not None and diff not in spliced:
                spliced.append(diff)
        return cls(primary=tuple(primary), alternates=tuple(spliced))

    def expand(self, index: int) -> tuple[int, ...]:
        """Materialise one alternative as a full route."""
        s = self.alternates[index]
        return self.primary[: s.start] + s.nodes + self.primary[s.end :]

    def surviving(self, blocked: frozenset[int]) -> tuple[int, ...]:
        """Indices of alternatives whose own stretch is still clear, cheapest first.

        Only the replacement nodes are checked. A blocked node the alternative
        shares with the primary kills the primary too, and that is a full replan
        rather than a promotion.
        """
        alive = [
            i for i, s in enumerate(self.alternates) if not (blocked & set(s.nodes))
        ]
        return tuple(sorted(alive, key=lambda i: (self.alternates[i].cost, i)))

    def promote(self, blocked: frozenset[int]) -> RouteCache | None:
        """Make the cheapest surviving alternative the primary, or None if none is.

        The rest are re-diffed against the new primary, so the cache stays a set
        of diffs rather than quietly becoming a set of full routes.
        """
        alive = self.surviving(blocked)
        if not alive:
            return None
        winner = alive[0]
        promoted = self.expand(winner)
        others = tuple(
            self.expand(i) for i in range(len(self.alternates)) if i != winner
        )
        costs = tuple(
            self.alternates[i].cost for i in range(len(self.alternates)) if i != winner
        )
        return RouteCache.build(promoted, others, costs, limit=len(self.alternates))

    @property
    def stored_nodes(self) -> int:
        """Total node ids held — the primary plus every diff."""
        return len(self.primary) + sum(s.size for s in self.alternates)

    def __len__(self) -> int:
        return len(self.alternates)
