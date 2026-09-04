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

import math
from dataclasses import dataclass
from typing import Dict, Sequence, Tuple

# Node kinds. Two letters so the payload stays inside QR alphanumeric mode.
KINDS = {
    "PT": "pass-through — a waypoint, confirms location, no action",
    "TR": "transfer — cargo pickup/dropoff",
    "CH": "charging bay",
    "PK": "parking — where an idle robot waits",
    "YI": "yield — a spur off the lane so one robot can let another pass",
}

# QR alphanumeric mode covers 0-9 A-Z space and $%*+-./: only. Separators are
# picked from that set so the code stays in alphanumeric rather than falling
# back to byte mode, which would cost roughly 45% more modules for the same
# payload -- and modules are pixels the camera has to resolve.
FIELD_SEP = "."
EDGE_SEP = "/"
BEARING_SEP = ":"

ORANGE = (255, 122, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


@dataclass(frozen=True)
class MarkerSpec:
    """Tile geometry in millimetres, plus the texture's resolution."""

    tile_mm: float = 100.0
    qr_mm: float = 60.0
    margin_mm: float = 5.0      # white quiet space between border and code
    pixels_per_mm: float = 5.12
    border_rgb: Tuple[int, int, int] = ORANGE

    @property
    def size_px(self) -> int:
        return round(self.tile_mm * self.pixels_per_mm)

    @property
    def border_mm(self) -> float:
        return (self.tile_mm - self.qr_mm) / 2.0 - self.margin_mm

    def modules_per_pixel(self, module_count: int) -> float:
        """Camera pixels per QR module, given a camera's mm/px."""
        return self.qr_mm / module_count


def encode_payload(
    node_id: int,
    kind: str,
    x_mm: int,
    y_mm: int,
    bearing_deg: int = 0,
    out_edges: Sequence[Tuple[int, int]] = (),
) -> str:
    """Build the marker's payload string.

    Carries absolute position, so the fleet's graph stays emergent: a robot
    reading a stream of these has observed the topology without ever being
    handed a map. `out_edges` is (bearing_deg, neighbour_id) pairs -- degree
    decides behaviour, so one out-edge is a waypoint and three is a junction,
    and no separate "junction" kind is needed.
    """
    if kind not in KINDS:
        raise ValueError("unknown node kind {!r}; expected one of {}".format(
            kind, ", ".join(sorted(KINDS))))
    if x_mm < 0 or y_mm < 0:
        raise ValueError(
            "marker coordinates must be non-negative ({}, {}); place the "
            "facility origin at a corner so no payload needs a sign".format(x_mm, y_mm))

    edges = EDGE_SEP.join(
        "{}{}{}".format(int(bearing) % 360, BEARING_SEP, int(neighbour))
        for bearing, neighbour in out_edges
    )
    # The tile's own bearing in the facility frame. The marker is laid along
    # the lane, so a robot that can measure the code's rotation in its camera
    # gets an absolute heading out of it -- which is what stops a lever-arm
    # fix from inheriting the odometry's accumulated heading drift.
    return FIELD_SEP.join([str(node_id), kind, str(int(x_mm)), str(int(y_mm)),
                           str(int(bearing_deg) % 360), edges])


def decode_payload(payload: str) -> Dict:
    """Inverse of `encode_payload`. Raises ValueError on anything malformed."""
    parts = payload.split(FIELD_SEP)
    if len(parts) != 6:
        raise ValueError("expected 6 fields, got {}: {!r}".format(len(parts), payload))

    node_id, kind, x_mm, y_mm, tile_bearing, edges = parts
    if kind not in KINDS:
        raise ValueError("unknown node kind {!r}".format(kind))

    out_edges = []
    if edges:
        for token in edges.split(EDGE_SEP):
            bearing, _, neighbour = token.partition(BEARING_SEP)
            if not neighbour:
                raise ValueError("malformed edge {!r} in {!r}".format(token, payload))
            out_edges.append((int(bearing), int(neighbour)))

    return {
        "node_id": int(node_id),
        "kind": kind,
        "x_mm": int(x_mm),
        "y_mm": int(y_mm),
        "bearing_deg": int(tile_bearing) % 360,
        "out_edges": out_edges,
    }


def qr_matrix(payload: str, error: str = "M") -> Tuple[Tuple[int, ...], ...]:
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
