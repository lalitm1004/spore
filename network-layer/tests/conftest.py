"""pytest setup shared by every test module.

WHAT / WHY
    * Puts the project root on sys.path so `import bot` works from anywhere.
    * Pins the environment `config.py` reads at import time — identity for
      the default Bot(), and *shorter timeouts* so migration-failure tests
      finish in seconds rather than tens of seconds. This must run before
      any project module is imported, which is exactly when conftest runs.
    * Provides the `fleet` harness fixture: builds bots on unique ports,
      starts their servers, and — crucially — stops every sender thread and
      server on teardown so tests cannot leak threads into each other.

HOW — ports
    Allocated from 21000 upwards, *below* Linux's ephemeral range
    (32768–60999). Ports inside that range can be grabbed by an earlier
    test's outgoing connection; gRPC then reports the bind failure by
    returning 0 rather than raising, and the server "starts" listening on
    nothing. `start_server` checks for that too.

HOW — addresses
    Tests dial 127.0.0.1, never "localhost": the latter resolves to ::1
    first, the servers bind IPv4 only, and a fresh channel's first RPC can
    fail on the refused ::1 before falling back. Docker's DNS returns IPv4
    only, so production never sees this — tests must avoid it explicitly.
"""
from __future__ import annotations

import os
import sys
import time
from concurrent import futures
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ.setdefault("BOT_ID", "0")
os.environ.setdefault("REGION_ID", "14")
os.environ.setdefault("GRPC_PORT", "50051")
os.environ.setdefault("OWN_ADDRESS", "127.0.0.1:50051")
os.environ.setdefault("PEER_LEADERS", "")
os.environ.setdefault("T_MIGRATION_TIMEOUT", "3.0")   # keep failure paths quick
os.environ.setdefault("T_MIGRATION_BACKOFF_MAX", "2.0")

import grpc  # noqa: E402
import pytest  # noqa: E402

from bot import Bot  # noqa: E402
from bus.policy import VirtualNetworkInterceptor, rpc_metadata  # noqa: E402
from bus.heartbeat import RegionServicer  # noqa: E402
from bus.jobs import JobServicer, BotServicer  # noqa: E402
from bus.leader_exchange import LeaderExchangeServicer  # noqa: E402
from bus.migration import MigrationJoinServicer  # noqa: E402
from election import priority as prio  # noqa: E402
from election.bully import Role  # noqa: E402
from election.server import ElectionServicer  # noqa: E402
from proto import fleet_pb2_grpc  # noqa: E402

_next_port = 21000


def next_port() -> int:
    global _next_port
    _next_port += 1
    return _next_port


def md(bot_id: int, region_id: int, role: str = "follower") -> list[tuple[str, str]]:
    """Identity metadata for hand-rolled RPCs — what a real bot attaches itself."""
    return rpc_metadata(bot_id, region_id, role)


def wait_until(pred, timeout: float = 10.0, step: float = 0.05) -> bool:
    """Poll `pred()` until true or `timeout` elapses. Returns the final value."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return bool(pred())


def make_bot(bot_id: int, port: int, region_id: int = 14, state: str = "IDLE") -> Bot:
    """A Bot with explicit identity (bypassing env) and a priority computed
    the way the run loop would (healthy @100% → 10400 + id, so ordering by
    id is preserved for readable tests)."""
    b = Bot()
    b.bot_id = bot_id
    b.region_id = region_id
    b.address = f"127.0.0.1:{port}"
    b.state = state
    b.priority = prio.compute(healthy=b.is_healthy(), battery_pct=b.battery, bot_id=bot_id)
    b.election.bot_id = bot_id
    b.election.priority = b.priority
    b.election.address = b.address
    return b


def start_server(bot: Bot, port: int) -> grpc.Server:
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=8),
        interceptors=[VirtualNetworkInterceptor(bot)],
    )
    fleet_pb2_grpc.add_ElectionServiceServicer_to_server(ElectionServicer(bot.election, bot.peer_table), server)
    fleet_pb2_grpc.add_RegionServiceServicer_to_server(RegionServicer(bot), server)
    fleet_pb2_grpc.add_LeaderExchangeServiceServicer_to_server(LeaderExchangeServicer(bot), server)
    fleet_pb2_grpc.add_MigrationJoinServiceServicer_to_server(MigrationJoinServicer(bot), server)
    fleet_pb2_grpc.add_JobServiceServicer_to_server(JobServicer(bot), server)
    fleet_pb2_grpc.add_BotServiceServicer_to_server(BotServicer(bot), server)
    if server.add_insecure_port(f"0.0.0.0:{port}") == 0:
        raise RuntimeError(f"could not bind test server to port {port}")
    server.start()
    return server


class Fleet:
    """Builds bots + servers and tears everything down afterwards."""

    def __init__(self) -> None:
        self.bots: list[Bot] = []
        self.servers: dict[int, grpc.Server] = {}
        self.ports: dict[int, int] = {}

    def bot(self, bot_id: int, region_id: int = 14, state: str = "IDLE", serve: bool = True) -> Bot:
        port = next_port()
        b = make_bot(bot_id, port, region_id, state)
        self.bots.append(b)
        self.ports[bot_id] = port
        if serve:
            self.servers[bot_id] = start_server(b, port)
        return b

    def serve(self, b: Bot) -> None:
        self.servers[b.bot_id] = start_server(b, self.ports[b.bot_id])

    def stop_server(self, b: Bot) -> None:
        s = self.servers.pop(b.bot_id, None)
        if s:
            s.stop(0)

    def link_all(self) -> None:
        """Every bot's PEER_LEADERS = every other bot (what up.py injects)."""
        for b in self.bots:
            b.peer_leaders = [o.address for o in self.bots if o is not b]

    def close(self) -> None:
        for b in self.bots:
            b.election.departing = True
            b._hb_sender.stop()
            b._leader_exchange.stop()
        for s in self.servers.values():
            s.stop(0)


@pytest.fixture
def fleet():
    f = Fleet()
    yield f
    f.close()


__all__ = ["Fleet", "Role", "fleet", "make_bot", "md", "next_port", "start_server", "wait_until"]
