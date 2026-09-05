"""One bot's view of who holds what.

Every bot keeps its own ledger; there is no shared table anywhere. It holds the
claims this bot has made and the claims its neighbours have announced, and answers
the only question that matters at the moment of moving: *may I enter this node
now?*

The answer is yes when this bot holds an effective claim covering the window and no
neighbour's claim overlaps it. Note the second half is deliberately stricter than
the arbitration rule needs -- a bot does not enter a contested node merely because
it outranks the other claimant. It waits for the loser's withdrawal to actually
arrive. Acting on "I should win" rather than "the conflict is gone" would mean
driving into a node whose other claimant has not yet heard it lost, which is
precisely the collision the protocol exists to prevent. The wait is bounded: the
loser withdraws as soon as it sees the contest, and its next announcement carries
the retraction.

Claims expire. A neighbour that stops announcing stops blocking after `ttl_ms`,
and prediction from its last known heading takes over -- degraded, but never
silently absent.
"""

from __future__ import annotations

from spore_planner.p2p.claims import Announcement, Claim, Contest, Window, contests
from spore_planner.planner.types import Reservation


class Ledger:
    """Per-bot reservation state. Holds no lock; drive it from one thread."""

    __slots__ = (
        "_expiry",
        "_mine",
        "_peers",
        "_seq",
        "announce_period_ms",
        "bot_id",
        "rank",
        "ttl_ms",
    )

    def __init__(
        self,
        bot_id: int,
        *,
        announce_period_ms: int = 200,
        ttl_ms: int = 600,
    ) -> None:
        self.bot_id = bot_id
        self.announce_period_ms = announce_period_ms
        self.ttl_ms = ttl_ms
        self.rank = 0
        self._mine: tuple[Claim, ...] = ()
        self._peers: dict[int, tuple[Claim, ...]] = {}
        self._expiry: dict[int, int] = {}
        self._seq = 0

    # -- this bot's own claims ----------------------------------------------

    def propose(
        self, windows: list[tuple[int, int, int]], now: int, *, rank: int | None = None
    ) -> bool:
        """Claim `(node_id, start_ms, end_ms)` windows, all or nothing.

        Refused if any neighbour already holds an overlapping claim this bot does
        not outrank. A partial grant would let a robot set off down a route it
        cannot finish, so the whole set stands or falls together.
        """
        if rank is not None:
            self.rank = rank
        candidate = tuple(
            Claim(
                bot_id=self.bot_id,
                node_id=node_id,
                start_ms=start_ms,
                end_ms=end_ms,
                rank=self.rank,
                effective_at_ms=now + self.announce_period_ms,
            )
            for node_id, start_ms, end_ms in windows
        )
        for contest in contests(candidate, self.peer_claims()):
            if contest.i_yield:
                return False
        self._mine = candidate
        self._seq += 1
        return True

    def withdraw(self) -> None:
        """Drop this bot's claims -- after losing a contest, or on replanning."""
        self._mine = ()
        self._seq += 1

    @property
    def mine(self) -> tuple[Claim, ...]:
        return self._mine

    def announcement(self, now: int) -> Announcement:
        """This bot's claims as relative offsets, ready to send."""
        return Announcement(
            bot_id=self.bot_id,
            seq=self._seq,
            rank=self.rank,
            ttl_ms=self.ttl_ms,
            windows=tuple(
                Window(
                    node_id=claim.node_id,
                    start_offset_ms=claim.start_ms - now,
                    end_offset_ms=claim.end_ms - now,
                )
                for claim in self._mine
            ),
        )

    # -- neighbours ----------------------------------------------------------

    def receive(self, announcement: Announcement, now: int) -> None:
        """Take a neighbour's announcement, stamped with local arrival time.

        An empty announcement is a withdrawal, not a lapse: the neighbour is
        saying it holds nothing, which frees its nodes at once rather than after
        the time-to-live.
        """
        if announcement.bot_id == self.bot_id:
            return
        claims = announcement.stamp(
            received_at_ms=now, effective_at_ms=now + self.announce_period_ms
        )
        self._peers[announcement.bot_id] = claims
        self._expiry[announcement.bot_id] = now + (announcement.ttl_ms or self.ttl_ms)

    def forget(self, bot_id: int) -> None:
        """Drop a neighbour outright -- it departed, or left the vicinity."""
        self._peers.pop(bot_id, None)
        self._expiry.pop(bot_id, None)

    def expire(self, now: int) -> tuple[int, ...]:
        """Drop neighbours that have gone quiet. Returns their bot ids."""
        stale = tuple(sorted(b for b, when in self._expiry.items() if when <= now))
        for bot_id in stale:
            self.forget(bot_id)
        return stale

    def peer_claims(self) -> tuple[Claim, ...]:
        return tuple(
            claim
            for bot_id in sorted(self._peers)
            for claim in self._peers[bot_id]
        )

    @property
    def neighbours(self) -> tuple[int, ...]:
        return tuple(sorted(self._peers))

    # -- the question that matters ------------------------------------------

    def may_enter(self, node_id: int, start_ms: int, end_ms: int, now: int) -> bool:
        """Whether this bot may drive into `node_id` for `[start_ms, end_ms]`."""
        held = any(
            claim.node_id == node_id
            and claim.start_ms <= start_ms
            and claim.end_ms >= end_ms
            and claim.effective_at_ms <= now
            for claim in self._mine
        )
        if not held:
            return False
        return not any(
            claim.overlaps(node_id, start_ms, end_ms) for claim in self.peer_claims()
        )

    def lost(self) -> tuple[Contest, ...]:
        """Contests this bot must give way in -- a signal to withdraw and replan."""
        return tuple(c for c in contests(self._mine, self.peer_claims()) if c.i_yield)

    def blockers(self, node_id: int, start_ms: int, end_ms: int) -> tuple[int, ...]:
        return tuple(
            sorted(
                {
                    claim.bot_id
                    for claim in self.peer_claims()
                    if claim.overlaps(node_id, start_ms, end_ms)
                }
            )
        )

    # -- handing the planner what it expects ---------------------------------

    def reservations_by_bot(self) -> dict[int, tuple[Reservation, ...]]:
        """Neighbour claims as planner `Reservation`s, already on the local clock.

        The planner's clock-offset and desync handling is unused here by design:
        relative windows arrive already correct, so there is nothing to correct.
        """
        return {
            bot_id: tuple(
                Reservation(node_id=c.node_id, t_in=c.start_ms, t_out=c.end_ms)
                for c in claims
            )
            for bot_id, claims in sorted(self._peers.items())
        }

    def __repr__(self) -> str:
        return (
            f"<Ledger bot-{self.bot_id} holding {len(self._mine)} "
            f"vs {len(self._peers)} neighbours>"
        )
