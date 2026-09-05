"""The warehouse map, reduced to the one thing the control plane can trust.

WHAT
    Loads `warehouse-layout.json` (schema: `warehouse-map.schema.json`) and
    answers a single question: *does node N exist?* (`has_node`).

WHERE
    Used by the web layer to reject a typo'd node id before it reaches the
    fleet. Nothing else.

WHY
    The control plane must not reason about geography: it can never know which
    region a bot is in (bots migrate), and node -> region is the fleet's
    internal concern. So the map is used only as a list of valid node ids, not
    for routing.

HOW
    A thin wrapper over the JSON. If the file is missing we degrade to
    `NullMap` (everything "exists") so the control plane still starts and
    still dispatches — the fleet routes regardless.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

log = logging.getLogger(__name__)


class NullMap:
    """Stand-in when no map file is available: accept every node id."""

    def has_node(self, node_id: int) -> bool:
        return True


class WarehouseMap:
    def __init__(self, data: dict) -> None:
        self._node_ids: set[int] = {n["id"] for n in data["nodes"]}

    @classmethod
    def load(cls, path: str | Path) -> "WarehouseMap | NullMap":
        p = Path(path)
        if not p.is_file():
            log.warning("warehouse map %s not found; node validation disabled", p)
            return NullMap()
        with p.open() as f:
            data = json.load(f)
        m = cls(data)
        log.info("warehouse map loaded: %d nodes", len(m._node_ids))
        return m

    def has_node(self, node_id: int) -> bool:
        return node_id in self._node_ids


__all__ = ["WarehouseMap", "NullMap"]
