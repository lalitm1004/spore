"""What a claim is, and who gives way when two of them collide.

WHAT
    * `Claim` — one bot's intent to hold one node for a window of time, and the
      ordering that settles a collision between two of them.
    * `Window` / `Announce` — the same thing as it travels on the wire, where
      times are offsets rather than instants.
    * `contests` — every overlap between two sets of claims.

WHERE
    Pure: no I/O, no protobuf, no clock. `reservations.ledger` holds these,
    `reservations.server` converts them to and from `fleet.proto` messages, and
    `reservations.sender` puts them on the wire (PROTOCOL.md §15).

WHY
    Claims travel bot to bot rather than through the leader, because §7 says
    collision avoidance must keep working when there is no leader. That rules
    out asking permission — there is nobody to ask — so the ordering here has to
    be something *both* bots can evaluate alone and agree on. It is built only
    from facts both already have: the yield priority the leader distributes in
    every `PeerRecord`, and the bot ids.

HOW
    * `order` is `(-rank, bot_id)`, lowest wins: a bot carrying cargo is never
      asked to give way to an idle one, and the id breaks the tie. Any total
      order would do; what matters is that it is computed from shared facts.
    * `overlaps` requires strictly positive overlap, so one bot may take a node
      the instant another releases it.
    * `effective_at_ms` is when a claim stops being provisional. See `ledger`.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Claim:
    """A hold on one node, in absolute local milliseconds."""

    bot_id: int
    node_id: int
    start_ms: int
    end_ms: int
    #: Right of way, straight from `election.priority.yield_priority`:
    #: 0 free, 1 heading to a pickup, 2 carrying cargo. Higher wins.
    rank: int = 0
    #: Before this instant the claim is provisional and must not be acted on.
    effective_at_ms: int = 0

    def overlaps(self, node_id: int, start_ms: int, end_ms: int) -> bool:
        return self.node_id == node_id and self.start_ms < end_ms and self.end_ms > start_ms

    @property
    def order(self) -> tuple[int, int]:
        """Arbitration key — lower wins. Both sides compute this identically."""
        return (-self.rank, self.bot_id)

    def outranks(self, other: Claim) -> bool:
        return self.order < other.order


@dataclass(frozen=True)
class Window:
    """A claim on the wire: offsets from the moment of sending, not instants."""

    node_id: int
    start_offset_ms: int
    end_offset_ms: int


@dataclass(frozen=True)
class Announce:
    """Everything one bot currently holds, as told to a neighbour.

    Deliberately carries no timestamp. The receiver stamps arrival, which is what
    lets two bots exchange claims without their clocks agreeing.
    """

    bot_id: int
    seq: int
    rank: int = 0
    #: How long these claims stay believable without a fresher announcement.
    ttl_ms: int = 0
    windows: tuple[Window, ...] = field(default_factory=tuple)

    def stamp(self, received_at_ms: int, effective_at_ms: int) -> tuple[Claim, ...]:
        """Turn wire offsets into claims on the receiver's own clock."""
        return tuple(
            Claim(
                bot_id=self.bot_id,
                node_id=w.node_id,
                start_ms=received_at_ms + w.start_offset_ms,
                end_ms=received_at_ms + w.end_offset_ms,
                rank=self.rank,
                effective_at_ms=effective_at_ms,
            )
            for w in self.windows
        )


@dataclass(frozen=True)
class Contest:
    """Two claims that cannot both stand, and who the ordering says loses."""

    node_id: int
    mine: Claim
    theirs: Claim

    @property
    def i_yield(self) -> bool:
        return self.theirs.outranks(self.mine)


def contests(mine: tuple[Claim, ...], theirs: tuple[Claim, ...]) -> tuple[Contest, ...]:
    """Every overlap between two claim sets, in a stable order.

    Sorted rather than incidental: both bots log and act on these, and a
    difference in order between the two sides would be a difference in behaviour.
    """
    by_node: dict[int, list[Claim]] = {}
    for claim in theirs:
        by_node.setdefault(claim.node_id, []).append(claim)

    found: list[Contest] = []
    for claim in sorted(mine, key=lambda c: (c.node_id, c.start_ms)):
        for other in sorted(by_node.get(claim.node_id, ()), key=lambda c: (c.start_ms, c.bot_id)):
            if other.bot_id != claim.bot_id and other.overlaps(
                claim.node_id, claim.start_ms, claim.end_ms
            ):
                found.append(Contest(node_id=claim.node_id, mine=claim, theirs=other))
    return tuple(found)
