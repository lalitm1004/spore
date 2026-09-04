"""Loading and validation for `fleet.yaml`, the single source of truth."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from tools.track.centerline import oval
from tools.track.marker import KINDS, MarkerSpec, encode_payload

SHAPES = {"oval": oval}


@dataclass(frozen=True)
class TrackConfig:
    """Ground plane and the track drawn on it.

    The plane and the track are sized independently so the plane can stay square
    (and therefore a power of two in pixels, which Webots does not rescale)
    while the track itself is free to be a non-square oval.
    """

    plane_size: Tuple[float, float]
    track_size: Tuple[float, float]
    shape: str = "oval"
    line_width: float = 0.02
    pixels_per_metre: int = 512

    @classmethod
    def from_dict(cls, data: dict) -> "TrackConfig":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        for key in ("plane_size", "track_size"):
            known[key] = tuple(float(v) for v in known[key])
        return cls(**known)

    def build_centerline(self):
        if self.shape not in SHAPES:
            raise ValueError(
                "unknown track shape {!r}; known shapes: {}".format(
                    self.shape, ", ".join(sorted(SHAPES))
                )
            )

        for axis, (track, plane) in enumerate(zip(self.track_size, self.plane_size)):
            if track + self.line_width > plane:
                raise ValueError(
                    "track {} does not fit on the plane along axis {}: "
                    "{} plus a {} line exceeds {}".format(
                        self.track_size, axis, track, self.line_width, plane
                    )
                )

        return SHAPES[self.shape](width=self.track_size[0], height=self.track_size[1])


@dataclass(frozen=True)
class MarkerNode:
    """One floor marker: a QR tile at a known point on the track.

    Placed by normalised arclength `at` along the centerline, so a marker
    always lands on the line rather than near it, and moving the track moves
    its markers with it.
    """

    node_id: int
    kind: str
    at: float
    out_edges: Tuple[Tuple[int, int], ...] = ()

    @classmethod
    def from_dict(cls, data: dict) -> "MarkerNode":
        kind = data["kind"]
        if kind not in KINDS:
            raise ValueError(
                "marker {} has unknown kind {!r}; known kinds: {}".format(
                    data.get("id"), kind, ", ".join(sorted(KINDS))))

        edges = tuple(
            (int(edge["bearing"]), int(edge["to"]))
            for edge in (data.get("out_edges") or [])
        )
        return cls(node_id=int(data["id"]), kind=kind, at=float(data["at"]), out_edges=edges)

    def world_pose(self, centerline) -> Tuple[float, float, float]:
        """(x, y, heading) in metres and radians."""
        x, y = centerline.point_at(self.at)
        return x, y, centerline.heading_at(self.at)

    def payload(self, centerline, origin_offset: Tuple[float, float]) -> str:
        """The QR's contents.

        Marker coordinates are absolute millimetres from a facility origin at
        the plane's corner, so every payload is non-negative and the fleet's
        graph stays emergent -- a robot reading these has observed the layout
        without ever being handed a map.
        """
        import math

        x, y, heading = self.world_pose(centerline)
        return encode_payload(
            node_id=self.node_id,
            kind=self.kind,
            x_mm=round((x + origin_offset[0]) * 1000),
            y_mm=round((y + origin_offset[1]) * 1000),
            bearing_deg=round(math.degrees(heading)) % 360,
            out_edges=self.out_edges,
        )


@dataclass(frozen=True)
class MarkerConfig:
    """Marker geometry plus the nodes laid out on the track."""

    spec: MarkerSpec = field(default_factory=MarkerSpec)
    nodes: Tuple[MarkerNode, ...] = ()

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> "MarkerConfig":
        if not data:
            return cls()

        spec_fields = {
            key: value for key, value in (data.get("spec") or {}).items()
            if key in MarkerSpec.__dataclass_fields__
        }
        if "border_rgb" in spec_fields:
            spec_fields["border_rgb"] = tuple(int(v) for v in spec_fields["border_rgb"])
        spec = MarkerSpec(**spec_fields)

        nodes: List[MarkerNode] = [MarkerNode.from_dict(n) for n in (data.get("nodes") or [])]

        seen = set()
        for node in nodes:
            if node.node_id in seen:
                raise ValueError("duplicate marker id {}".format(node.node_id))
            seen.add(node.node_id)

        known = {node.node_id for node in nodes}
        for node in nodes:
            for _, neighbour in node.out_edges:
                if neighbour not in known:
                    raise ValueError(
                        "marker {} points at unknown node {}".format(node.node_id, neighbour))

        return cls(spec=spec, nodes=tuple(nodes))


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` onto `base`, mutating neither."""
    merged = dict(base)
    for key, value in override.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = deep_merge(existing, value)
        else:
            merged[key] = value
    return merged
