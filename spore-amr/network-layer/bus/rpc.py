"""Persistent gRPC channels — one per target address, shared by every sender.

Opening a channel per RPC meant a TCP + HTTP/2 handshake for every heartbeat
(30 followers × 1 Hz = 30 handshakes/s on the leader) and exposed us to the
fresh-channel race where the first RPC fails before the connection settles.
gRPC channels reconnect on their own, so keeping one alive per peer is both
cheaper and more reliable.

Reconnect backoff is capped low: a peer that comes back after a crash must be
reachable again within ~T_HB, not after gRPC's default 2-minute backoff.
"""
from __future__ import annotations

import threading

import grpc

import config

_OPTIONS = [
    ("grpc.initial_reconnect_backoff_ms", int(config.T_RECONNECT_MIN * 1000)),
    ("grpc.min_reconnect_backoff_ms", int(config.T_RECONNECT_MIN * 1000)),
    ("grpc.max_reconnect_backoff_ms", int(config.T_RECONNECT_MAX * 1000)),
    ("grpc.keepalive_time_ms", int(config.T_KEEPALIVE * 1000)),
    ("grpc.keepalive_timeout_ms", int(config.T_KEEPALIVE_TIMEOUT * 1000)),
    ("grpc.keepalive_permit_without_calls", 1),
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

    def drop(self, address: str) -> None:
        """Forget a peer (e.g. it departed) so we stop holding a socket to it."""
        with self._lock:
            ch = self._channels.pop(address, None)
        if ch is not None:
            ch.close()

    def close_all(self) -> None:
        with self._lock:
            chans = list(self._channels.values())
            self._channels.clear()
        for ch in chans:
            ch.close()


pool = ChannelPool()


def stub(address: str, stub_cls):
    """Convenience: `stub(addr, fleet_pb2_grpc.RegionServiceStub).Heartbeat(...)`."""
    return pool.stub(address, stub_cls)
