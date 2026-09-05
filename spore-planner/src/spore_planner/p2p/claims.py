"""Claims, announcements, and the rule that decides who gives way.

Reservations live on a peer plane rather than in the leader's heartbeat, because
the protocol requires that collision avoidance keep working when the leader is
gone (PROTOCOL.md §7). Two design choices follow from that, and both are what make
this safe without a round trip.

**Announce, don't negotiate.** A bot broadcasts what it intends to hold; it does
not ask. When two claims collide, both bots can see both claims and both apply the
same ordering, so they reach the same answer independently. Asking permission would
need a reply per claim and would deadlock the moment two robots asked at once.

**Relative windows.** A claim says "I hold node 412 from +200 ms to +2400 ms", and
the receiver stamps its own clock on arrival. No shared clock, no beacon, no
drift correction -- the error is one network hop, far below the safety margin the
planner already adds. Absolute timestamps would have dragged clock synchronisation
into the middle of collision avoidance.

**A claim is not usable the instant it is made.** Two bots that claim the same node
in the same breath have not yet heard each other, and if either acted immediately
they would collide before the conflict was visible. So a fresh claim is
*provisional*; it becomes *effective* only after one announce period, by which time
any competing claim has arrived and the loser has withdrawn. At 5 Hz that is 200 ms
of latency against a two-second hop -- cheap insurance, and it is what makes an
announce-only protocol correct rather than merely optimistic.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True, order=False)
class Claim:
    """One bot's intent to hold one node for a window of local time.

    Times are absolute milliseconds on the *holder's* clock once received and
    stamped; they travel as offsets (see `Announcement`).
    """

    bot_id: int
    node_id: int
    start_ms: int
    end_ms: int
    rank: int = 0
    """Right of way, from the leader's `yield_priority`: 0 free, 1 heading to a
    pickup, 2 carrying cargo. Higher wins."""

    effective_at_ms: int = 0
    """When this claim stops being provisional. Before it, the holder must not
    move into the node -- a competing claim may still be in flight."""

    def overlaps(self, node_id: int, start_ms: int, end_ms: int) -> bool:
        """Whether this claim contests `[start_ms, end_ms]` on `node_id`.

        Overlap must be strictly positive: one robot may take a node from the
        exact instant another releases it, which is the same convention the
        planner's safe intervals use.
        """
        return self.node_id == node_id and self.start_ms < end_ms and self.end_ms > start_ms

    @property
    def order(self) -> tuple[int, int]:
        """Sort key for arbitration -- lower wins.

        Rank first, so a robot carrying cargo is not asked to give way to an idle
        one. Then bot id, purely to break the tie the same way on both sides. Any
        total order would do; what matters is that every bot computes it
        identically from data every bot already has.
        """
        return (-self.rank, self.bot_id)

    def outranks(self, other: Claim) -> bool:
        return self.order < other.order


@dataclass(frozen=True, slots=True)
class Window:
    """A claim as it goes on the wire: offsets from the moment of sending."""

    node_id: int
    start_offset_ms: int
    end_offset_ms: int


@dataclass(frozen=True, slots=True)
class Announcement:
    """What one bot tells its neighbours it intends to hold.

    Carries no timestamp on purpose. The receiver stamps arrival, which is what
    keeps the two bots from needing a shared clock.
    """

    bot_id: int
    seq: int
    rank: int = 0
    ttl_ms: int = 0
    """How long these claims stay believable without a fresher announcement. A bot
    that goes quiet stops blocking its neighbours once this lapses, and they fall
    back to predicting its motion from its last known heading."""

    windows: tuple[Window, ...] = ()

    def stamp(self, received_at_ms: int, effective_at_ms: int) -> tuple[Claim, ...]:
        """Turn wire offsets into absolute claims on the receiver's clock."""
        return tuple(
            Claim(
                bot_id=self.bot_id,
                node_id=window.node_id,
                start_ms=received_at_ms + window.start_offset_ms,
                end_ms=received_at_ms + window.end_offset_ms,
                rank=self.rank,
                effective_at_ms=effective_at_ms,
            )
            for window in self.windows
        )


@dataclass(frozen=True, slots=True)
class Contest:
    """A collision between two claims, and who the rule says gives way."""

    node_id: int
    mine: Claim
    theirs: Claim

    @property
    def i_yield(self) -> bool:
        return self.theirs.outranks(self.mine)


def contests(
    mine: tuple[Claim, ...], theirs: tuple[Claim, ...]
) -> tuple[Contest, ...]:
    """Every overlap between two sets of claims, in deterministic order."""
    found: list[Contest] = []
    by_node: dict[int, list[Claim]] = {}
    for claim in theirs:
        by_node.setdefault(claim.node_id, []).append(claim)
    for claim in sorted(mine, key=lambda c: (c.node_id, c.start_ms)):
        for other in sorted(by_node.get(claim.node_id, ()), key=lambda c: (c.start_ms, c.bot_id)):
            if other.bot_id != claim.bot_id and other.overlaps(
                claim.node_id, claim.start_ms, claim.end_ms
            ):
                found.append(Contest(node_id=claim.node_id, mine=claim, theirs=other))
    return tuple(found)
