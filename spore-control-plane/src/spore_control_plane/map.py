"""The warehouse map, reduced to what the control plane needs.

WHAT
    Loads `warehouse-layout.json` (schema: `warehouse-map.schema.json`) and
    answers two questions:
      * does node N exist?                     (`has_node`)
      * which region is node N in?             (`region_of`)
      * what nodes are in region R?            (`nodes_in` — for tests / UI hints)

WHERE
    Used by the web layer to validate order node ids and to resolve
    `pickup_node -> region_id`, which is how dispatch picks the target leader.

WHY
    The control plane does not do path planning and does not need hop
    distances. Positions alone mislead in a warehouse, but we only need the
    node -> region mapping, which is a straight lookup.

HOW
    A thin wrapper over the JSON. If the file is missing we degrade to
    `NullMap` (nothing validates, `region_of` returns None) so the control
    plane still starts and can still dispatch — the fleet will route an order
    to the right region regardless.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

log = logging.getLogger(__name__)


class NullMap:
    """Stand-in when no map file is available. See module docstring."""

    def has_node(self, node_id: int) -> bool:
        return True

    def region_of(self, node_id: int) -> int | None:
        return None

    def nodes_in(self, region_id: int) -> list[int]:
        return []


class WarehouseMap:
    def __init__(self, data: dict) -> None:
        self._region_of: dict[int, int] = {n["id"]: n["region_id"] for n in data["nodes"]}
        self._by_region: dict[int, list[int]] = {}
        for node_id, region in self._region_of.items():
            self._by_region.setdefault(region, []).append(node_id)

    @classmethod
    def load(cls, path: str | Path) -> "WarehouseMap | NullMap":
        p = Path(path)
        if not p.is_file():
            log.warning("warehouse map %s not found; node validation disabled", p)
            return NullMap()
        with p.open() as f:
            data = json.load(f)
        m = cls(data)
        log.info("warehouse map loaded: %d nodes, %d regions", len(m._region_of), len(m._by_region))
        return m

    def has_node(self, node_id: int) -> bool:
        return node_id in self._region_of

    def region_of(self, node_id: int) -> int | None:
        return self._region_of.get(node_id)

    def nodes_in(self, region_id: int) -> list[int]:
        return list(self._by_region.get(region_id, []))


__all__ = ["WarehouseMap", "NullMap"]
