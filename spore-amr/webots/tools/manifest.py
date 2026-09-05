"""Loading and validation for `fleet.yaml`, the single source of truth."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from tools.track.centerline import oval
from tools.track.graph import lattice
from tools.track.marker import KINDS, MarkerSpec, encode_payload

# Name prefixes follow warehouse.json's convention, e.g. "charging/PT/001".
KIND_SLUGS = {"PT": "aisle", "TR": "transfer", "CH": "charging",
              "PK": "parking", "YI": "yield"}

SHAPES = {"oval": oval}


@dataclass(frozen=True)
class GraphConfig:
    """A lattice track: the shape that has junctions.

    `spacing` defaults to 2.0 m to match warehouse.json's node_spacing of
    200 cm, so the simulated geometry is the real geometry.
    """

    rows: int = 4
    columns: int = 4
    spacing: float = 2.0
    kinds: Tuple[Tuple[int, int, str], ...] = ()   # (row, column, kind)

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> Optional["GraphConfig"]:
        if not data:
            return None
        kinds = tuple(
            (int(k["row"]), int(k["column"]), str(k["kind"]))
            for k in (data.get("kinds") or [])
        )
        return cls(
            rows=int(data.get("rows", 4)),
            columns=int(data.get("columns", 4)),
            spacing=float(data.get("spacing", 2.0)),
            kinds=kinds,
        )

    def build(self):
        return lattice(rows=self.rows, columns=self.columns, spacing=self.spacing,
                       kinds={(r, c): k for r, c, k in self.kinds})

    def required_plane(self) -> float:
        """Smallest plane that fits the lattice with a margin, rounded up to a
        power of two in metres so the texture is never rescaled."""
        span = max((self.rows - 1), (self.columns - 1)) * self.spacing
        needed = span + 2.0
        size = 1.0
        while size < needed:
            size *= 2.0
        return size


@dataclass(frozen=True)
class WarehouseConfig:
    """A window of a real warehouse.json, used as the track.

    The full 120 x 70 m layout cannot be simulated at line-following
    resolution -- see tools/track/warehouse.load_window for the arithmetic --
    so a window of it is the faithful option: real ids, real types, real edges.
    """

    source: str
    origin_cm: Tuple[float, float]
    size_m: Tuple[float, float]
    pixels_per_metre: int = 256
    # Floor beyond the outermost nodes. The window used to end exactly where
    # the graph did, so a boundary node had no floor past it: a robot routed
    # there ran out of world. Off the plane every IR sensor reads black_ref,
    # which the estimator renders as a perfectly centred line at maximum
    # confidence, and one robot drove 18 m into the void believing it was on
    # the lane. The margin is what a robot overshooting a node lands on.
    margin_m: float = 3.0

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> Optional["WarehouseConfig"]:
        if not data:
            return None
        return cls(
            source=str(data["source"]),
            origin_cm=tuple(float(v) for v in data["origin_cm"]),
            size_m=tuple(float(v) for v in data["size_m"]),
            pixels_per_metre=int(data.get("pixels_per_metre", 256)),
            margin_m=float(data.get("margin_m", 3.0)),
        )

    def build(self):
        from tools.track.warehouse import load_window

        return load_window(self.source, self.origin_cm, self.size_m)

    @property
    def plane_size(self) -> Tuple[float, float]:
        """The floor, which is the window plus a margin on every side."""
        return (self.size_m[0] + 2 * self.margin_m,
                self.size_m[1] + 2 * self.margin_m)

    @property
    def plane_origin_cm(self) -> Tuple[float, float]:
        return (self.origin_cm[0] - self.margin_m * 100.0,
                self.origin_cm[1] - self.margin_m * 100.0)


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
    graph: Optional[GraphConfig] = None
    warehouse: Optional[WarehouseConfig] = None

    @classmethod
    def from_dict(cls, data: dict) -> "TrackConfig":
        known = {f: data[f] for f in cls.__dataclass_fields__
                 if f in data and f not in ("graph", "warehouse")}
        warehouse = WarehouseConfig.from_dict(data.get("warehouse"))
        if warehouse is not None:
            # The plane is bigger than the window: the graph fills the window,
            # and the margin is floor for a robot that overshoots its edge.
            known["plane_size"] = warehouse.plane_size
            known["track_size"] = warehouse.size_m
            known["shape"] = "warehouse"
            known["pixels_per_metre"] = warehouse.pixels_per_metre
            for key in ("plane_size", "track_size"):
                known[key] = tuple(float(v) for v in known[key])
            return cls(graph=None, warehouse=warehouse, **known)

        graph = GraphConfig.from_dict(data.get("graph"))
        if graph is not None:
            # The plane follows the lattice, not the other way round: it has to
            # be a power of two in metres or Webots rescales the texture.
            size = graph.required_plane()
            known["plane_size"] = (size, size)
            known.setdefault("track_size", (size, size))
            known["shape"] = "lattice"
        for key in ("plane_size", "track_size"):
            if key in known:
                known[key] = tuple(float(v) for v in known[key])
        return cls(graph=graph, warehouse=None, **known)

    @property
    def is_graph(self) -> bool:
        """True for any node-and-lane track, lattice or real warehouse."""
        return self.graph is not None or self.warehouse is not None

    def build_graph(self):
        if self.warehouse is not None:
            return self.warehouse.build()
        if self.graph is None:
            raise ValueError("this track has no graph; it is a {}".format(self.shape))
        return self.graph.build()

    @property
    def node_spacing_cm(self) -> int:
        if self.warehouse is not None:
            return 200
        return int(self.graph.spacing * 100) if self.graph else 200

    def build_centerline(self):
        if self.is_graph:
            raise ValueError(
                "a lattice track has no single centerline; use build_graph()")
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
    name: str = ""
    region_id: int = 0

    @classmethod
    def from_dict(cls, data: dict) -> "MarkerNode":
        kind = data["kind"]
        if kind not in KINDS:
            raise ValueError(
                "marker {} has unknown kind {!r}; known kinds: {}".format(
                    data.get("id"), kind, ", ".join(sorted(KINDS))))

        node_id = int(data["id"])
        return cls(
            node_id=node_id,
            kind=kind,
            at=float(data["at"]),
            # The schema requires a name. Follow warehouse.json's convention
            # rather than inventing one, so a marker generated here is
            # indistinguishable from one generated there.
            name=data.get("name") or "{}/{}/{:03d}".format(
                KIND_SLUGS.get(kind, "node"), kind, node_id),
            region_id=int(data.get("region_id", 0)),
        )

    def world_pose(self, centerline) -> Tuple[float, float, float]:
        """(x, y, heading) in metres and radians."""
        x, y = centerline.point_at(self.at)
        return x, y, centerline.heading_at(self.at)

    def payload(self, centerline, origin_offset: Tuple[float, float]) -> str:
        """The QR's contents, per the shared schema.

        Coordinates are centimetres from a facility origin at the plane's
        corner, matching `warehouse.json`'s `units`. The payload carries no
        bearing and no out-edges: both are derivable from the shared map, and
        every character omitted is modules the camera does not have to resolve.
        """
        x, y, _ = self.world_pose(centerline)
        return encode_payload(
            node_id=self.node_id,
            kind=self.kind,
            x_cm=round((x + origin_offset[0]) * 100, 1),
            y_cm=round((y + origin_offset[1]) * 100, 1),
            name=self.name,
            region_id=self.region_id,
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
