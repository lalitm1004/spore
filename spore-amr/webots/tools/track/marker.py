"""Floor markers: a QR code inside a coloured border, rendered as one tile.

Each marker is its own texture on its own small plane, deliberately *not* part
of the track texture. At the track's 512 px/m a QR module would be one texel
wide with no antialiasing headroom, and Webots would mipmap the finder patterns
into mush -- the same class of failure as the non-power-of-two rescale that
resampled the line edges. A dedicated tile decouples marker resolution from
track resolution entirely, and costs one small PNG per node.

The coloured border is what the 1x1 colour camera sees. It exists so the QR
camera can stay dark until a read is worth paying for, and it has to be wide
enough to survive sampling: at cruise (0.12 m/s) and a 16 ms control step, a
15 mm band is about 8 samples.

Pure: renders images and encodes payloads, no I/O and no Webots.
"""

import json
import math
from dataclasses import dataclass
from typing import Dict, Tuple

# Node kinds. Two letters so the payload stays inside QR alphanumeric mode.
KINDS = {
    "PT": "pass-through — a waypoint, confirms location, no action",
    "TR": "transfer — cargo pickup/dropoff",
    "CH": "charging bay",
    "PK": "parking — where an idle robot waits",
    "YI": "yield — a spur off the lane so one robot can let another pass",
}

# The shared schema is JSON, so the code is in byte mode and runs to 57 modules
# rather than the 33 a compact string managed. That is the cost of one contract
# across the whole system instead of two, and it is paid in camera resolution:
# 57 modules across a 60 mm code needs 512 px to keep 4 px per module.
SCHEMA_VERSION = "v0.1.0"

ORANGE = (255, 122, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


@dataclass(frozen=True)
class MarkerSpec:
    """Tile geometry in millimetres, plus the texture's resolution."""

    tile_mm: float = 100.0
    qr_mm: float = 60.0
    margin_mm: float = 5.0      # white quiet space between border and code
    # Chosen so tile_mm * pixels_per_mm lands on a power of two: 100 mm at
    # 10.24 px/mm is exactly 1024. Webots rescales a non-power-of-two texture
    # silently, and rescaling a QR code resamples the very module edges the
    # decoder reads -- the same failure that once resampled the lane edges the
    # IR array reads.
    pixels_per_mm: float = 10.24
    border_rgb: Tuple[int, int, int] = ORANGE

    @property
    def size_px(self) -> int:
        return round(self.tile_mm * self.pixels_per_mm)

    @property
    def border_mm(self) -> float:
        return (self.tile_mm - self.qr_mm) / 2.0 - self.margin_mm

def encode_payload(
    node_id: int,
    kind: str,
    x_cm: float,
    y_cm: float,
    name: str,
    region_id: int = 0,
    schema_version: str = SCHEMA_VERSION,
) -> str:
    """Build the marker's payload: one Node, per the shared QR schema.

    The schema lives at spore-amr/shared/schemas/qr-code.schema.json and is the
    contract with the network layer, so this follows it exactly rather than
    carrying anything convenient.

    Notably it carries no out-edges and no lane bearing, and it does not need
    to: the warehouse map is a generated artifact every robot holds, so both
    are derivable from the node id. The marker only has to answer "which node
    am I on", and the smaller it can say that, the more reliably it decodes.

    Coordinates are centimetres, matching `warehouse.json`'s `units`.
    """
    if kind not in KINDS:
        raise ValueError("unknown node kind {!r}; expected one of {}".format(
            kind, ", ".join(sorted(KINDS))))
    if not name:
        raise ValueError("node name is required by the schema")

    document = {
        "schema_version": schema_version,
        "data": {
            "id": int(node_id),
            "name": name,
            "region_id": int(region_id),
            "node_type": kind,
            "position": {"x": float(x_cm), "y": float(y_cm)},
        },
    }
    # Compact separators: every character costs modules, and modules are the
    # camera pixels this has to survive being sampled into.
    return json.dumps(document, separators=(",", ":"))


def decode_payload(payload: str) -> Dict:
    """Inverse of `encode_payload`. Raises ValueError on anything malformed."""
    try:
        document = json.loads(payload)
    except ValueError as error:
        raise ValueError("payload is not JSON: {}".format(error))

    if not isinstance(document, dict) or "data" not in document:
        raise ValueError("payload has no `data` object: {!r}".format(payload[:60]))

    data = document["data"]
    missing = [key for key in ("id", "name", "region_id", "node_type", "position")
               if key not in data]
    if missing:
        raise ValueError("node is missing {}".format(", ".join(missing)))

    kind = data["node_type"]
    if kind not in KINDS:
        raise ValueError("unknown node kind {!r}".format(kind))

    position = data["position"]
    if "x" not in position or "y" not in position:
        raise ValueError("position needs both x and y")

    return {
        "schema_version": document.get("schema_version", ""),
        "node_id": int(data["id"]),
        "name": data["name"],
        "region_id": int(data["region_id"]),
        "kind": kind,
        "x_cm": float(position["x"]),
        "y_cm": float(position["y"]),
    }


# Error correction L, not M. Two reasons, and both point the same way:
#
#   * L needs 49 modules for a node payload where M needs 57, and modules are
#     camera pixels this has to survive being sampled into.
#   * OpenCV's decoder is measurably less reliable on the larger code -- 4 of 5
#     node payloads at M, 5 of 5 at L, on the same renders.
#
# The recovery L gives up is bought back in time instead: a marker is in view
# for roughly 22 frames, so a frame that fails is simply not the frame that
# gets used.
ERROR_CORRECTION = "L"


def qr_matrix(payload: str, error: str = ERROR_CORRECTION) -> Tuple[Tuple[int, ...], ...]:
    """QR modules including the mandatory 4-module quiet zone."""
    import segno

    code = segno.make(payload, error=error)
    rows = [tuple(int(bit) for bit in row) for row in code.matrix]
    width = len(rows[0])

    quiet = 4
    blank = tuple([0] * (width + 2 * quiet))
    padded = [blank] * quiet
    for row in rows:
        padded.append(tuple([0] * quiet + list(row) + [0] * quiet))
    padded.extend([blank] * quiet)
    return tuple(padded)


def render_marker(payload: str, spec: MarkerSpec = MarkerSpec()) -> "Image.Image":
    """Draw one marker tile: coloured border, white quiet space, QR in the middle."""
    from PIL import Image, ImageDraw

    size = spec.size_px
    image = Image.new("RGB", (size, size), spec.border_rgb)
    draw = ImageDraw.Draw(image)

    border_px = spec.border_mm * spec.pixels_per_mm
    draw.rectangle(
        [border_px, border_px, size - border_px - 1, size - border_px - 1],
        fill=WHITE,
    )

    matrix = qr_matrix(payload)
    modules = len(matrix)
    qr_px = spec.qr_mm * spec.pixels_per_mm
    scale = qr_px / modules
    origin = (size - qr_px) / 2.0

    for row_index, row in enumerate(matrix):
        for col_index, bit in enumerate(row):
            if not bit:
                continue
            x0 = origin + col_index * scale
            y0 = origin + row_index * scale
            # Ceil the far edge so adjacent dark modules never leave a seam of
            # background between them after rounding.
            draw.rectangle(
                [math.floor(x0), math.floor(y0),
                 math.ceil(x0 + scale) - 1, math.ceil(y0 + scale) - 1],
                fill=BLACK,
            )

    return image
