"""Virtual network — application-level isolation on top of a flat physical network.

All bots share one Docker network (and IRL one WiFi), so isolation can't come
from the transport. Instead every client call carries identity metadata and a
server interceptor enforces who may call which service:

    RegionService, ElectionService   caller must be in *our* region
    LeaderExchangeService            caller must be a leader (any region)
    MigrationJoinService             caller must have a pending handoff with us
    JobService (SubmitJob)           any authenticated caller — the order
                                     system or any bot may hand in a job
    BotService (AssignJob)           caller must be a leader
    AdminService                     only when ADMIN_ENABLED (local / tests)

Calls without metadata are UNAUTHENTICATED; disallowed calls are
PERMISSION_DENIED. Both surface to the client as grpc.RpcError, which the
senders already treat as "peer unreachable".
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)

MD_BOT_ID = "bot-id"
MD_REGION_ID = "region-id"
MD_ROLE = "role"


def rpc_metadata(bot_id: int, region_id: int, role) -> list[tuple[str, str]]:
    """Metadata every outgoing call must attach. `role` is a Role enum or str."""
    role_name = role.name if hasattr(role, "name") else str(role)
    return [
        (MD_BOT_ID, str(bot_id)),
        (MD_REGION_ID, str(region_id)),
        (MD_ROLE, role_name.lower()),
    ]


def _deny(code: grpc.StatusCode, msg: str):
    def handler(request, context):
        context.abort(code, msg)

    return grpc.unary_unary_rpc_method_handler(handler)


class VirtualNetworkInterceptor(grpc.ServerInterceptor):
    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def intercept_service(self, continuation, details):
        md = dict(details.invocation_metadata)
        service = details.method.split("/")[1] if details.method.count("/") >= 2 else ""

        try:
            caller_id = int(md[MD_BOT_ID])
            caller_region = int(md[MD_REGION_ID])
            caller_role = md[MD_ROLE]
        except (KeyError, ValueError):
            log.warning(
                "bot-%d: %s called without fleet metadata", self._bot.bot_id, details.method
            )
            return _deny(grpc.StatusCode.UNAUTHENTICATED, "missing fleet metadata")

        if not self._allowed(service, caller_id, caller_region, caller_role):
            log.warning(
                "bot-%d: denied %s from bot-%d (region=%d, role=%s)",
                self._bot.bot_id,
                details.method,
                caller_id,
                caller_region,
                caller_role,
            )
            return _deny(grpc.StatusCode.PERMISSION_DENIED, "not on this virtual network")

        return continuation(details)

    def _allowed(self, service: str, caller_id: int, caller_region: int, caller_role: str) -> bool:
        bot = self._bot
        if service in ("fleet.RegionService", "fleet.ElectionService"):
            return caller_region == bot.region_id
        if service == "fleet.LeaderExchangeService":
            return caller_role == "leader"
        if service == "fleet.MigrationJoinService":
            return caller_id in bot.pending_incoming
        if service == "fleet.JobService":
            return True  # authenticated is enough; routing decides the rest
        if service == "fleet.BotService":
            return caller_role == "leader"
        if service == "fleet.AdminService":
            import config
            return config.ADMIN_ENABLED
        return False
