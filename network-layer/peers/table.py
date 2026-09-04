"""Thread-safe bookkeeping: who is in my region, who leads the others, and
which bots are mid-migration.

WHAT
    Three small containers:
      * `PeerTable` — the region roster (`Peer` records) plus the cached
        `Leader` record for every other region.
      * `Ledger`   — a bot_id → payload map with TTL expiry, used for both
        the leader's `migrating_out` set and the destination leader's
        `pending_incoming` set.
      * The `Peer` / `Leader` dataclasses themselves.

WHERE
    Owned by `bot.Bot`. Written by gRPC handlers (leader side: heartbeats,
    departures, joins) and by the heartbeat sender (follower side: full
    replacement from each ack). Read by the election, the leader exchange,
    and the run loop.

WHY
    The roster is the *only* state the protocol keeps, and every bot keeps a
    copy (PROTOCOL.md §7: the leader must never hold the only copy). It is
    touched from many threads, so all access goes through one lock.

    Migration bookkeeping lives *outside* the roster on purpose: a follower's
    heartbeat rewrites its `Peer` record every second, so a "MIGRATING_OUT"
    flag stored on the record would be overwritten within one T_HB. Keeping
    it in a separate `Ledger` makes it survive until Departure or timeout.

HOW
    Plain dicts under a `threading.Lock`. Followers call `replace()` to swap
    the whole roster atomically (no empty window between clear and refill in
    which an election could start against nobody and win by default).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any

from config import T_DEAD
from election.priority import is_healthy


@dataclass
class Peer:
    """One bot in my region, as last reported (by its heartbeat, or by the
    leader's ack if I am a follower).

    `last_seen` is monotonic time and is only meaningful on the leader, where
    it drives eviction; followers refresh it wholesale on every ack.
    """

    bot_id: int
    address: str
    priority: int
    state: str = "IDLE"
    battery: float = 100.0
    latest_node_id: int = 0
    #: Recent QR nodes, newest first; node_trail[0] == latest_node_id.
    node_trail: list[int] = field(default_factory=list)
    #: What the bot is doing and whether it is in trouble (from its heartbeat).
    mission: str = ""
    fault: str = ""
    #: The job it is executing, if any (travels with the bot across regions).
    job_id: str = ""
    cargo_state: str = ""
    last_seen: float = field(default_factory=time.monotonic)


@dataclass
class Leader:
    """The leader of some *other* region, learned through the leader exchange
    (or relayed to followers in HeartbeatAck.other_leaders)."""

    region_id: int
    bot_id: int
    address: str


class PeerTable:
    """Region roster + other-region leaders, safe to use from any thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._peers: dict[int, Peer] = {}
        self._other_leaders: dict[int, Leader] = {}
        #: region_id → {bot_id → node_trail}: where bots in *other* regions
        #: have been lately, learned from leader heartbeats (leaders only).
        self._region_locations: dict[int, dict[int, list[int]]] = {}

    # ---- Roster ----------------------------------------------------------

    def upsert(self, peer: Peer) -> None:
        """Insert or refresh one peer and stamp it as seen now (leader side)."""
        with self._lock:
            peer.last_seen = time.monotonic()
            self._peers[peer.bot_id] = peer

    def ensure(self, bot_id: int, address: str, priority: int) -> None:
        """Make sure `bot_id` is in the roster with this address/priority
        without disturbing anything else we know about it.

        Used when a bot *contacts* us during an election: it is alive and in
        our region, so it belongs in the set our Coordinator goes to — even
        if it was missing from the last ack we saw.
        """
        with self._lock:
            existing = self._peers.get(bot_id)
            if existing is None:
                self._peers[bot_id] = Peer(bot_id=bot_id, address=address, priority=priority)
            else:
                existing.address = address
                existing.priority = priority
                existing.last_seen = time.monotonic()

    def remove(self, bot_id: int) -> None:
        with self._lock:
            self._peers.pop(bot_id, None)

    def get(self, bot_id: int) -> Peer | None:
        with self._lock:
            return self._peers.get(bot_id)

    def all_peers(self) -> list[Peer]:
        with self._lock:
            return list(self._peers.values())

    def replace(self, peers: list[Peer], leaders: list[Leader]) -> None:
        """Atomically swap the whole view (follower side, on every ack).

        Doing this as clear()+upsert() leaves a window where the roster is
        empty; an election starting in that window finds no higher peers and
        declares itself leader. One assignment under the lock avoids that.
        """
        now = time.monotonic()
        with self._lock:
            for p in peers:
                p.last_seen = now
            self._peers = {p.bot_id: p for p in peers}
            self._other_leaders = {ld.region_id: ld for ld in leaders}

    def evict_dead(self, ttl: float = T_DEAD) -> list[Peer]:
        """Drop peers not heard from within `ttl` seconds (leader side, each
        tick). Returns the evicted records — the dispatcher needs to know
        whether a vanished bot was carrying a job."""
        now = time.monotonic()
        evicted: list[Peer] = []
        with self._lock:
            for bot_id, peer in list(self._peers.items()):
                if now - peer.last_seen > ttl:
                    del self._peers[bot_id]
                    evicted.append(peer)
        return evicted

    def highest_priority_peer(self, exclude: int | None = None) -> Peer | None:
        """The peer that would win an election. `(priority, bot_id)` ordering
        matches the bully rule."""
        with self._lock:
            candidates = [p for p in self._peers.values() if p.bot_id != exclude]
            if not candidates:
                return None
            return max(candidates, key=lambda p: (p.priority, p.bot_id))

    def best_successor(self, exclude: int | None = None) -> Peer | None:
        """Who a leader should hand off to (migration, fault, tenure,
        shutdown): a healthy bot, preferably one with NO job — a busy bot
        would drive away and have to hand off again — then by election
        priority. Falls back to a busy bot if every healthy peer is busy;
        never to an unhealthy one."""
        with self._lock:
            candidates = [p for p in self._peers.values()
                          if p.bot_id != exclude and is_healthy(p.state)]
            if not candidates:
                return None
            return max(candidates, key=lambda p: (not p.job_id, p.priority, p.bot_id))

    # ---- Other regions' leaders -----------------------------------------

    def set_leaders(self, leaders: list[Leader]) -> None:
        with self._lock:
            self._other_leaders = {ld.region_id: ld for ld in leaders}

    def upsert_leader(self, leader: Leader) -> None:
        """Keyed by region: a new leader for a region simply replaces the old
        record, which is how other regions learn about a succession."""
        with self._lock:
            self._other_leaders[leader.region_id] = leader

    def remove_leader(self, region_id: int) -> None:
        with self._lock:
            self._other_leaders.pop(region_id, None)

    def get_leader(self, region_id: int) -> Leader | None:
        with self._lock:
            return self._other_leaders.get(region_id)

    def all_leaders(self) -> list[Leader]:
        with self._lock:
            return list(self._other_leaders.values())

    # ---- Other regions' bot locations (leaders only) -------------------

    def set_region_locations(self, region_id: int, trails: dict[int, list[int]]) -> None:
        """Replace what we know about where region `region_id`'s bots are."""
        with self._lock:
            self._region_locations[region_id] = dict(trails)

    def region_locations(self) -> dict[int, dict[int, list[int]]]:
        """Snapshot: region_id → {bot_id → node_trail} for every other region."""
        with self._lock:
            return {r: dict(t) for r, t in self._region_locations.items()}

    # ---- Misc -------------------------------------------------------------

    def clear(self) -> None:
        with self._lock:
            self._peers.clear()
            self._other_leaders.clear()
            self._region_locations.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._peers)


class Ledger:
    """bot_id → (payload, marked_at) with TTL expiry. Thread-safe.

    Used for the two migration bookkeeping sets that must *not* be part of
    the roster (see module docstring):

      * source leader's  `migrating_out`   payload = destination region id
      * destination's    `pending_incoming` payload = source region id

    Supports `bot_id in ledger`, which is what the virtual-network policy
    uses to admit a MigrationJoin.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._entries: dict[int, tuple[Any, float]] = {}

    def mark(self, bot_id: int, payload: Any = None) -> None:
        with self._lock:
            self._entries[bot_id] = (payload, time.monotonic())

    def get(self, bot_id: int) -> Any | None:
        with self._lock:
            entry = self._entries.get(bot_id)
            return entry[0] if entry else None

    def pop(self, bot_id: int) -> Any | None:
        with self._lock:
            entry = self._entries.pop(bot_id, None)
            return entry[0] if entry else None

    def expire(self, ttl: float) -> list[int]:
        """Drop entries older than `ttl`; return their ids. This is how each
        party enforces T_MIGRATION_TIMEOUT independently (PROTOCOL.md §8)."""
        now = time.monotonic()
        expired: list[int] = []
        with self._lock:
            for bot_id, (_, marked_at) in list(self._entries.items()):
                if now - marked_at > ttl:
                    del self._entries[bot_id]
                    expired.append(bot_id)
        return expired

    def ids(self) -> list[int]:
        with self._lock:
            return list(self._entries)

    def __contains__(self, bot_id: int) -> bool:
        with self._lock:
            return bot_id in self._entries

    def __len__(self) -> int:
        with self._lock:
            return len(self._entries)
