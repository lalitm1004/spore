"""Leader discovery: learn each region's leader by asking the fleet.

WHAT
    `LeaderDirectory` maintains a `region_id -> LeaderInfo` map, populated by
    calling `DiscoverLeaders` on the bots we know, with a short TTL so a
    leadership change is picked up quickly. A cache miss triggers a refresh.

WHERE
    Used by the web layer to route an order straight to the leader of the
    pickup node's region. The submitter falls back to "any bot" when this
    directory is empty or stale, so discovery is an optimization, not a
    dependency.

WHY
    The control plane is told where every bot is (it boots the fleet) but is
    deliberately *not* told who leads. The only honest way to learn leaders is
    to ask a bot; every bot can answer `DiscoverLeaders` with what it knows.

HOW
    * `refresh()` fans out to every `BOT_ADDRESSES`, merges the results (later
      answers win), and records the answering bot's own region/leader.
    * `leader_for(region_id)` returns a cached entry if fresh, otherwise
      refreshes once and re-checks. TTL is `LEADER_CACHE_TTL`.
"""
from __future__ import annotations

import logging
import time

import grpc

from spore_control_plane import config
from spore_control_plane import client
from spore_control_plane.proto import controlplane_pb2, controlplane_pb2_grpc

log = logging.getLogger(__name__)


class LeaderDirectory:
    def __init__(self, addresses: list[str] | None = None) -> None:
        self._addresses = list(addresses if addresses is not None else config.BOT_ADDRESSES)
        self._leaders: dict[int, controlplane_pb2.LeaderInfo] = {}
        self._updated_at: float = 0.0

    def leader_for(self, region_id: int) -> controlplane_pb2.LeaderInfo | None:
        """The leader of `region_id`, or None if unknown. Refreshes on a miss
        or when the cache is stale."""
        now = time.monotonic()
        if now - self._updated_at >= config.LEADER_CACHE_TTL:
            self.refresh()
        return self._leaders.get(region_id)

    def all(self) -> list[controlplane_pb2.LeaderInfo]:
        return list(self._leaders.values())

    def refresh(self) -> None:
        """Ask every known bot what it knows; merge. Nothing here raises — a
        dead bot is simply skipped, and we keep whatever we already had."""
        if not self._addresses:
            log.debug("no bot addresses configured; leader directory empty")
            return
        merged: dict[int, controlplane_pb2.LeaderInfo] = {}
        for address in self._addresses:
            self._merge(merged, self._query(address))
        self._leaders = merged
        self._updated_at = time.monotonic()
        log.info("leader directory refreshed: %d region(s) from %d bot(s)",
                 len(self._leaders), len(self._addresses))

    def _query(self, address: str) -> controlplane_pb2.DiscoverLeadersResponse | None:
        try:
            stub = client.pool.stub(address, controlplane_pb2_grpc.ControlPlaneServiceStub)
            return stub.DiscoverLeaders(
                controlplane_pb2.DiscoverLeadersRequest(),
                timeout=config.GRPC_TIMEOUT,
                metadata=client.metadata(),
            )
        except grpc.RpcError as e:
            log.debug("DiscoverLeaders to %s failed: %s", address, e.code())
            return None

    @staticmethod
    def _merge(target: dict[int, controlplane_pb2.LeaderInfo],
               resp: controlplane_pb2.DiscoverLeadersResponse | None) -> None:
        if resp is None:
            return
        for leader in resp.leaders:
            target[leader.region_id] = leader
        # The answering bot's own leader may not be in `leaders` (its own
        # region is "self", not "other"), so fold it in explicitly.
        if resp.self_region_id and resp.self_leader_address:
            target[resp.self_region_id] = controlplane_pb2.LeaderInfo(
                region_id=resp.self_region_id,
                bot_id=resp.self_leader_bot_id,
                address=resp.self_leader_address,
            )
