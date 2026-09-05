"""Marker payloads, rendering, and whether a camera can still read the result.

The payload follows the shared QR schema at
`spore-amr/shared/schemas/qr-code.schema.json`, so these tests assert the
contract, not a convenient local format.

The last test is the one that matters: a marker that renders beautifully at
texture resolution is worthless if the robot's camera samples it into mush.
"""

import json
import math

import pytest

from tools.track.marker import (
    ERROR_CORRECTION,
    MarkerSpec,
    decode_payload,
    encode_payload,
    qr_matrix,
    render_marker,
)

NODE = dict(node_id=20, kind="TR", x_cm=311.0, y_cm=120.8,
            name="transfer/TR/020", region_id=2)


# --------------------------------------------------------------- the schema --

def test_payload_matches_the_shared_schema():
    document = json.loads(encode_payload(**NODE))

    assert document == {
        "schema_version": "v0.1.0",
        "data": {
            "id": 20,
            "name": "transfer/TR/020",
            "region_id": 2,
            "node_type": "TR",
            "position": {"x": 311.0, "y": 120.8},
        },
    }


def test_payload_round_trips():
    fields = decode_payload(encode_payload(**NODE))

    assert fields["node_id"] == 20
    assert fields["name"] == "transfer/TR/020"
    assert fields["region_id"] == 2
    assert fields["kind"] == "TR"
    assert fields["x_cm"] == pytest.approx(311.0)
    assert fields["y_cm"] == pytest.approx(120.8)


def test_payload_uses_compact_separators():
    """Every character is modules the camera has to resolve."""
    payload = encode_payload(**NODE)
    assert ", " not in payload and '": ' not in payload


def test_unknown_kind_is_rejected():
    with pytest.raises(ValueError, match="unknown node kind"):
        encode_payload(node_id=1, kind="ZZ", x_cm=0, y_cm=0, name="x")


def test_name_is_required_by_the_schema():
    with pytest.raises(ValueError, match="name"):
        encode_payload(node_id=1, kind="PT", x_cm=0, y_cm=0, name="")


def test_negative_coordinates_are_allowed():
    """Unlike the earlier local format, the schema's position is a plain
    number -- the facility origin does not have to sit at a corner."""
    assert decode_payload(encode_payload(
        node_id=1, kind="PT", x_cm=-5.5, y_cm=-2.0, name="a"))["x_cm"] == -5.5


def test_malformed_payloads_are_rejected():
    for bad in ('not json',
                '{"schema_version":"v0.1.0"}',                       # no data
                '{"data":{"id":1}}',                                # missing fields
                '{"data":{"id":1,"name":"a","region_id":0,'
                '"node_type":"ZZ","position":{"x":0,"y":0}}}',      # bad kind
                '{"data":{"id":1,"name":"a","region_id":0,'
                '"node_type":"PT","position":{"x":0}}}'):           # no y
        with pytest.raises(ValueError):
            decode_payload(bad)


# ------------------------------------------------------------- the rendering --

def test_render_places_modules_where_the_matrix_says():
    """Sample each module's centre against the source matrix. Catches
    off-by-one scaling and origin errors without needing a decoder."""
    pytest.importorskip("PIL")
    payload = encode_payload(**NODE)
    spec = MarkerSpec()
    pixels = render_marker(payload, spec).load()

    matrix = qr_matrix(payload)
    modules = len(matrix)
    qr_px = spec.qr_mm * spec.pixels_per_mm
    scale = qr_px / modules
    origin = (spec.size_px - qr_px) / 2.0

    for row_index, row in enumerate(matrix):
        for col_index, bit in enumerate(row):
            x = int(origin + (col_index + 0.5) * scale)
            y = int(origin + (row_index + 0.5) * scale)
            assert (pixels[x, y][0] < 128) == bool(bit), \
                "module ({}, {}) wrong".format(row_index, col_index)


def test_border_is_the_trigger_colour_at_the_tile_edge():
    pytest.importorskip("PIL")
    spec = MarkerSpec()
    pixels = render_marker(encode_payload(**NODE), spec).load()

    assert pixels[2, 2] == spec.border_rgb
    assert pixels[spec.size_px // 2, spec.size_px // 2] in ((0, 0, 0), (255, 255, 255))


# ------------------------------------------------------------- the optics ----

def test_low_error_correction_keeps_the_code_small():
    """L, not M. A node payload is 49 modules at L and 57 at M, and OpenCV's
    decoder measurably prefers the smaller code -- 5 of 5 node payloads
    decoded at L against 4 of 5 at M on identical renders.

    The recovery given up is bought back in time: a marker is in view for
    roughly 22 frames, so a frame that fails is not the frame that gets used.
    """
    segno = pytest.importorskip("segno")
    assert ERROR_CORRECTION == "L"
    assert segno.make(encode_payload(**NODE), error="L").version <= 6


def test_survives_the_camera_resolution():
    """Crop a tile to what the camera really sees, resample to the sensor's
    pixel count, and decode it with the same OpenCV call that runs on the Pi.

    This is the test that decides whether the optics work at all. The shared
    schema is JSON, so a node payload is 49 modules where a compact string was
    33 -- which is exactly why the camera is 512 px and not 256.
    """
    Image = pytest.importorskip("PIL.Image", reason="Pillow required")
    pytest.importorskip("cv2", reason="OpenCV required")

    from robot.qr import QrReader, to_gray
    import numpy as np

    mast, wheel_radius, fov, resolution = 0.06, 0.02, 1.05, 512
    footprint_mm = 2 * (mast + wheel_radius) * math.tan(fov / 2) * 1000

    spec = MarkerSpec()
    payload = encode_payload(**NODE)
    tile = render_marker(payload, spec)

    crop_px = int(footprint_mm * spec.pixels_per_mm)
    inset = (spec.size_px - crop_px) // 2
    frame = tile.crop((inset, inset, inset + crop_px, inset + crop_px)) \
                .resize((resolution, resolution), Image.BILINEAR)

    modules = len(qr_matrix(payload))
    per_module = (spec.qr_mm / footprint_mm * resolution) / modules
    assert per_module >= 4.0, \
        "only {:.1f} px per module; decoding is unreliable".format(per_module)

    array = np.array(frame)
    bgra = np.dstack([array[:, :, 2], array[:, :, 1], array[:, :, 0],
                      np.full((resolution, resolution), 255, np.uint8)]).astype(np.uint8)
    read = QrReader().read(to_gray(bgra.tobytes(), resolution, resolution))

    assert read is not None, "the code did not decode at camera resolution"
    assert read.node_id == 20
    assert read.kind == "TR"
    assert read.x_cm == pytest.approx(311.0)
