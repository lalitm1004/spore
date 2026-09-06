"""Shared gRPC plumbing: the identity metadata and a persistent channel pool.

WHAT
    * `metadata()` — the reserved identity the control plane attaches to every
      outgoing call (the fleet's virtual network requires it).
    * `ChannelPool` — one channel per target address, shared by all callers,
      so we don't pay a TCP + HTTP/2 handshake per RPC.

WHERE
    Imported by `submitter`.

WHY
    gRPC channels reconnect on their own; keeping one alive per bot is cheaper
    and more reliable than opening one per call. The identity is the one thing
    every call must carry, so it lives here in a single place.

HOW
    A dict of `grpc.Channel` under a lock, keyed by address.
"""
from __future__ import annotations

import threading

import grpc

from spore_control_plane import config

# Reconnect quickly: a bot that comes back after a restart must be reachable
# again promptly, not after gRPC's default 2-minute backoff.
_OPTIONS = [
    ("grpc.initial_reconnect_backoff_ms", 200),
    ("grpc.min_reconnect_backoff_ms", 200),
    ("grpc.max_reconnect_backoff_ms", 2000),
    ("grpc.keepalive_time_ms", 10_000),
    ("grpc.keepalive_timeout_ms", 3_000),
    ("grpc.keepalive_permit_without_calls", 1),
]


def metadata() -> list[tuple[str, str]]:
    """The control plane's identity on the wire. The fleet's virtual network
    reads `bot-id` / `region-id` / `role` from this."""
    return [
        ("bot-id", str(config.CONTROL_BOT_ID)),
        ("region-id", str(config.CONTROL_REGION_ID)),
        ("role", config.CONTROL_ROLE),
    ]


class ChannelPool:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._channels: dict[str, grpc.Channel] = {}

    def channel(self, address: str) -> grpc.Channel:
        with self._lock:
            ch = self._channels.get(address)
            if ch is None:
                ch = grpc.insecure_channel(address, options=_OPTIONS)
                self._channels[address] = ch
            return ch

    def stub(self, address: str, stub_cls):
        return stub_cls(self.channel(address))

    def close_all(self) -> None:
        with self._lock:
            channels = list(self._channels.values())
            self._channels.clear()
        for ch in channels:
            ch.close()


pool = ChannelPool()
