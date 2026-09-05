"""Peer reservations, turned into the safe intervals the search plans through.

Reservations address nodes only, so an edge conflict has to be inferred rather than
read off. That works because of one invariant, which the reservation layer is
required to honour when it publishes windows:

    A robot holds *both* endpoint nodes for the whole duration of a traversal. It
    does not release A until it is fully inside B, so `t_out(A) >= t_in(B)`.

Given that, two robots swapping across an edge each claim both of its endpoints at
once, and the clash shows up as an ordinary overlap on a node. Following too closely
is caught the same way. So node-interval disjointness is the *only* test needed, and
this module's job is simply to compute, per node, the windows that are already
spoken for -- and hence the windows that are not.

Three things widen an incoming claim before it is stored:

* the clock offset, putting the peer's timestamps on the local clock;
* `skew_bound_ms`, for peers whose clocks are not trusted;
* `safety_ms`, covering control latency and speed-estimate error.

Widening is always outward. Overestimating a peer's occupancy costs throughput;
underestimating it risks a collision.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

from spore_planner.planner.types import INF_MS, Config, PeerView
from spore_planner.warehouse.graph import Graph
from spore_planner.warehouse.map import Heading

Interval = tuple[int, int]

_FOREVER: tuple[Interval, ...] = ()


class Claim:
    """One peer's widened, clock-corrected hold on a node."""

    __slots__ = ("bot_id", "dir", "end", "node", "start")

    def __init__(self, node: int, start: int, end: int, bot_id: int, dir: Heading | None):
        self.node = node
        self.start = start
        self.end = end
        self.bot_id = bot_id
        self.dir = dir

    def __repr__(self) -> str:
        return f"<Claim node={self.node} [{self.start},{self.end}] bot={self.bot_id}>"


class ReservationTable:
    """Per-node blocked and safe intervals, built once per plan call."""

    __slots__ = (
        "_blocked",
        "_claims",
        "_safe",
        "config",
        "graph",
        "now",
        "unknown_node_claims",
    )

    def __init__(
        self,
        graph: Graph,
        peers: Iterable[PeerView],
        *,
        now: int,
        config: Config,
        exclude_bot_id: int | None = None,
    ) -> None:
        self.graph = graph
        self.now = now
        self.config = config
        self.unknown_node_claims = 0

        claims: dict[int, list[Claim]] = {}
        # Sorted by bot id so that diagnostics and tie-breaks are reproducible.
        for peer in sorted(peers, key=lambda p: p.bot_id):
            if exclude_bot_id is not None and peer.bot_id == exclude_bot_id:
                continue
            margin = config.safety_ms + config.follow_gap_ms
            if peer.desynced:
                margin += config.skew_bound_ms
            for reservation in peer.reservations:
                if not graph.has_id(reservation.node_id):
                    # A peer running a different map revision. Ignoring the claim is
                    # the only option -- there is no node here to block -- but it is
                    # worth surfacing rather than swallowing.
                    self.unknown_node_claims += 1
                    continue
                node = graph.index(reservation.node_id)
                start = reservation.t_in + peer.clock_offset_ms - margin
                end = reservation.t_out + peer.clock_offset_ms + margin
                claims.setdefault(node, []).append(
                    Claim(node, start, end, peer.bot_id, reservation.dir)
                )

        self._claims: dict[int, tuple[Claim, ...]] = {
            node: tuple(sorted(items, key=lambda c: (c.start, c.end, c.bot_id)))
            for node, items in claims.items()
        }
        self._blocked: dict[int, tuple[Interval, ...]] = {
            node: _merge((c.start, c.end) for c in items)
            for node, items in self._claims.items()
        }
        self._safe: dict[int, tuple[Interval, ...]] = {
            node: _complement(blocked, now) for node, blocked in self._blocked.items()
        }

    # -- queries -------------------------------------------------------------

    def blocked(self, node: int) -> tuple[Interval, ...]:
        """Merged windows during which some peer holds `node`."""
        return self._blocked.get(node, _FOREVER)

    def safe_intervals(self, node: int) -> tuple[Interval, ...]:
        """Windows from `now` onwards during which `node` is free.

        Always at least one interval unless a peer has claimed the node forever.
        """
        cached = self._safe.get(node)
        if cached is None:
            return ((self.now, INF_MS),)
        return cached

    def claims(self, node: int) -> tuple[Claim, ...]:
        """Individual peer claims on `node`, for diagnostics."""
        return self._claims.get(node, ())

    def is_free(self, node: int, start: int, end: int) -> bool:
        """Whether `node` is unclaimed for the whole of `[start, end]`.

        Overlap has to be strictly positive to count, which is what makes this agree
        with the safe intervals the search plans through: a robot may claim a node
        from the exact instant another releases it. That handoff is not as tight as
        it looks -- every claim has already been widened by `safety_ms` at both ends,
        so the real clearance is that margin twice over.
        """
        for blocked_start, blocked_end in self.blocked(node):
            if blocked_start >= end:
                break
            if blocked_end > start:
                return False
        return True

    def blockers(self, node: int, start: int, end: int) -> tuple[int, ...]:
        """Bot ids whose claims overlap `[start, end]` on `node`."""
        found = {
            claim.bot_id
            for claim in self.claims(node)
            if claim.start < end and claim.end > start
        }
        return tuple(sorted(found))

    def interval_containing(self, node: int, t: int) -> Interval | None:
        """The safe interval holding instant `t`, if any."""
        for interval in self.safe_intervals(node):
            if interval[0] <= t <= interval[1]:
                return interval
            if interval[0] > t:
                break
        return None

    @property
    def blocked_nodes(self) -> frozenset[int]:
        return frozenset(self._blocked)

    def __repr__(self) -> str:
        return f"<ReservationTable {len(self._blocked)} blocked nodes at t={self.now}>"


def _merge(intervals: Iterable[Interval]) -> tuple[Interval, ...]:
    """Union of possibly overlapping intervals, sorted and coalesced.

    Intervals that merely touch are merged too: a node released at exactly the
    instant it is claimed again leaves no usable gap.
    """
    ordered: Sequence[Interval] = sorted(intervals)
    if not ordered:
        return ()
    merged: list[Interval] = [ordered[0]]
    for start, end in ordered[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end:
            if end > last_end:
                merged[-1] = (last_start, end)
        else:
            merged.append((start, end))
    return tuple(merged)


def _complement(blocked: Sequence[Interval], now: int) -> tuple[Interval, ...]:
    """Gaps between merged blocked intervals, from `now` to forever."""
    safe: list[Interval] = []
    cursor = now
    for start, end in blocked:
        if end < now:
            continue
        if start > cursor:
            safe.append((cursor, start))
        cursor = max(cursor, end)
    safe.append((cursor, INF_MS))
    return tuple(safe)
