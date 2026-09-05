"""Runtime configuration for the control plane.

WHAT
    Everything the control plane needs at boot: how to reach the fleet, the
    identity it presents on the wire, where the warehouse map lives, and the
    timing/bind knobs for the HTTP server and gRPC calls.

WHERE
    Read once at import time by every module. Treat as constants.

WHY
    The control plane is deployed as a container, so all of this comes from
    environment variables — the only thing it knows at birth. Defaults are
    chosen so a bare `uv run spore-control-plane` still starts (with an empty
    fleet and a degraded, geography-blind map) rather than crashing.

HOW
    Plain `os.environ` reads, no external config framework.
"""
from __future__ import annotations

import os
from pathlib import Path

# ---- Fleet reachability -----------------------------------------------------

def _csv(name: str, default: str = "") -> list[str]:
    return [part.strip() for part in os.environ.get(name, default).split(",") if part.strip()]


#: Addresses (host:port) of the bots the control plane may talk to. The
#: control plane is also the Kubernetes controller that boots the fleet, so it
#: already knows these. Order is only used as a fallback dispatch order.
BOT_ADDRESSES: list[str] = _csv("BOT_ADDRESSES")

# ---- Wire identity ----------------------------------------------------------

#: Reserved identity the control plane presents in gRPC metadata. The fleet's
#: virtual network requires `bot-id` / `region-id` / `role` on every call;
#: these values are deliberately outside the fleet's own id space (bots are
#: < 100) so the network layer can admit us by this id without mistaking us
#: for a robot. The exact role string the network layer admits is configurable.
CONTROL_BOT_ID = int(os.environ.get("CONTROL_BOT_ID", "9000"))
CONTROL_REGION_ID = int(os.environ.get("CONTROL_REGION_ID", "0"))
CONTROL_ROLE = os.environ.get("CONTROL_ROLE", "control")

# ---- Map --------------------------------------------------------------------

#: Path to warehouse-layout.json (the same map the fleet loads). Used only to
#: validate node ids and resolve pickup_node -> region; a missing file degrades
#: to "no validation, no region lookup" and dispatch still works.
WAREHOUSE_MAP = os.environ.get(
    "WAREHOUSE_MAP",
    str(Path(__file__).resolve().parent.parent.parent / "shared" / "warehouse-layout.json"),
)

# ---- Timing -----------------------------------------------------------------

#: Per-RPC timeout for DispatchOrder / DiscoverLeaders.
GRPC_TIMEOUT = float(os.environ.get("GRPC_TIMEOUT", "5.0"))

#: How long a leader-directory entry is trusted before a re-discovery. A miss
#: (unknown region) always triggers a refresh, regardless of TTL.
LEADER_CACHE_TTL = float(os.environ.get("LEADER_CACHE_TTL", "10.0"))

# ---- HTTP server ------------------------------------------------------------

HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8000"))
