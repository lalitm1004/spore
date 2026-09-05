"""One bot's record of who holds what.

WHAT
    `ReservationLedger` — this bot's own claims plus the claims its neighbours have
    announced, and the one question that matters at the moment of moving:
    *may I enter this node now?*

WHERE
    One per `bot.Bot`. `reservations.sender` proposes claims into it and
    publishes them; `reservations.server` feeds neighbours' announcements in.
    Pure: no I/O, no protobuf, no threads of its own.

WHY
    There is no shared table anywhere, by design (PROTOCOL.md §7 — collision
    avoidance must survive the leader). Every bot decides for itself from what it
    has been told, so the rules have to be ones that give both sides the same
    answer without a conversation.

HOW — two rules that look over-cautious and are not
    * **A fresh claim is provisional.** It only counts one announce period after
      it is made. Two bots that claim the same node in the same breath have not
      heard each other yet; if either acted at once they would collide before the
      clash was visible. Waiting one round guarantees it surfaces first.
    * **`may_enter` refuses while *any* neighbour claim overlaps**, even one this
      bot outranks. "I should win" is not "the other bot knows it lost". The
      loser withdraws as soon as it sees the contest and its next announcement
      carries the retraction; until then, driving in would hit a robot that has
      not got the message.

    Claims expire. A neighbour that stops announcing stops blocking after
    `ttl_ms`, so a crashed bot does not wedge a lane forever.
"""
from __future__ import annotations

from reservations.claims import Announce, Claim, Contest, Window, contests


class ReservationLedger:
    """Per-bot reservation state. Not thread-safe; drive it from the run loop."""

    def __init__(self, bot_id: int, *, announce_period_ms: int, ttl_ms: int) -> None:
        self.bot_id = bot_id
        self.announce_period_ms = announce_period_ms
        self.ttl_ms = ttl_ms
        self.rank = 0
        self._mine: tuple[Claim, ...] = ()
        self._peers: dict[int, tuple[Claim, ...]] = {}
        self._expiry: dict[int, int] = {}
        self._seen_seq: dict[int, int] = {}
        self._seq = 0

    # ---- Our own claims -----------------------------------------------------

    def propose(
        self, windows: list[tuple[int, int, int]], now: int, *, rank: int | None = None
    ) -> bool:
        """Claim `(node_id, start_ms, end_ms)` windows, all of them or none.

        A partial grant would let a robot set off down a route it cannot finish,
        so one refused window refuses the set.

        Carrying on holding a node keeps the original `effective_at_ms`. Only a
        genuinely new claim is provisional: the waiting period exists so that
        neighbours get a chance to object to a node being taken, and nobody new
        needs to object to a bot still sitting where it already was. Restamping
        on every tick would restart the clock forever and the claim would never
        come into force — quietly, because the run loop moves the robot before it
        re-claims, so nothing would look wrong.

        "Carrying on" means the same node with a window that runs on from the one
        we hold. A fresh window on the same node with a gap before it is a new
        claim and waits its turn like any other.
        """
        if rank is not None:
            self.rank = rank

        fresh = now + self.announce_period_ms
        candidate = tuple(
            Claim(
                bot_id=self.bot_id,
                node_id=node_id,
                start_ms=start_ms,
                end_ms=end_ms,
                rank=self.rank,
                effective_at_ms=self._continues(node_id, start_ms) or fresh,
            )
            for node_id, start_ms, end_ms in windows
        )
        if any(c.i_yield for c in contests(candidate, self.peer_claims())):
            return False
        self._mine = candidate
        self._seq += 1
        return True

    def _continues(self, node_id: int, start_ms: int) -> int | None:
        """The effective time of a claim this one carries on from, if any."""
        for claim in self._mine:
            if claim.node_id == node_id and claim.start_ms <= start_ms <= claim.end_ms:
                return claim.effective_at_ms
        return None

    def withdraw(self) -> None:
        """Drop our claims — after losing a contest, or when the route changes."""
        self._mine = ()
        self._seq += 1

    @property
    def mine(self) -> tuple[Claim, ...]:
        return self._mine

    def announcement(self, now: int) -> Announce:
        """Our claims as offsets from `now`, ready for the wire."""
        return Announce(
            bot_id=self.bot_id,
            seq=self._seq,
            rank=self.rank,
            ttl_ms=self.ttl_ms,
            windows=tuple(
                Window(
                    node_id=c.node_id,
                    start_offset_ms=c.start_ms - now,
                    end_offset_ms=c.end_ms - now,
                )
                for c in self._mine
            ),
        )

    # ---- Neighbours ---------------------------------------------------------

    def receive(self, announce: Announce, now: int) -> None:
        """Take a neighbour's announcement, stamped with local arrival time.

        An empty announcement is a *withdrawal*, not a lapse: the neighbour is
        saying it holds nothing, and its nodes free up at once rather than after
        the time to live.

        An announcement older than one we have already applied is dropped. Each
        one carries the sender's whole claim set, so applying a stale one would
        resurrect claims the sender has since given up — and reordering is
        possible whenever a channel reconnects.
        """
        if announce.bot_id == self.bot_id:
            return
        if announce.seq < self._seen_seq.get(announce.bot_id, -1):
            return
        self._seen_seq[announce.bot_id] = announce.seq
        self._peers[announce.bot_id] = announce.stamp(
            received_at_ms=now, effective_at_ms=now + self.announce_period_ms
        )
        self._expiry[announce.bot_id] = now + (announce.ttl_ms or self.ttl_ms)

    def forget(self, bot_id: int) -> None:
        """Drop a neighbour — it departed, or drifted out of claim range."""
        self._peers.pop(bot_id, None)
        self._expiry.pop(bot_id, None)
        self._seen_seq.pop(bot_id, None)

    def expire(self, now: int) -> tuple[int, ...]:
        """Drop neighbours that have gone quiet. Returns their bot ids."""
        stale = tuple(sorted(b for b, when in self._expiry.items() if when <= now))
        for bot_id in stale:
            self.forget(bot_id)
        return stale

    def peer_claims(self) -> tuple[Claim, ...]:
        return tuple(c for bot_id in sorted(self._peers) for c in self._peers[bot_id])

    @property
    def neighbours(self) -> tuple[int, ...]:
        return tuple(sorted(self._peers))

    # ---- The question that matters ------------------------------------------

    def may_enter(self, node_id: int, start_ms: int, end_ms: int, now: int) -> bool:
        """May this bot drive into `node_id` for `[start_ms, end_ms]`?"""
        held = any(
            c.node_id == node_id
            and c.start_ms <= start_ms
            and c.end_ms >= end_ms
            and c.effective_at_ms <= now
            for c in self._mine
        )
        if not held:
            return False
        return not any(c.overlaps(node_id, start_ms, end_ms) for c in self.peer_claims())

    def lost(self) -> tuple[Contest, ...]:
        """Contests we must give way in — the signal to withdraw and replan."""
        return tuple(c for c in contests(self._mine, self.peer_claims()) if c.i_yield)

    def blockers(self, node_id: int, start_ms: int, end_ms: int) -> tuple[int, ...]:
        return tuple(
            sorted({c.bot_id for c in self.peer_claims() if c.overlaps(node_id, start_ms, end_ms)})
        )

    def __repr__(self) -> str:
        return (
            f"<ReservationLedger bot-{self.bot_id} holding {len(self._mine)} "
            f"vs {len(self._peers)} neighbours>"
        )
