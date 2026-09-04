"""Marker payloads, rendering, and whether a camera can still read the result.

The last of those is the one that matters: a marker that renders beautifully at
texture resolution is worthless if the robot's camera samples it into mush.
"""

import math

import pytest

from tools.track.marker import (
    MarkerSpec,
    decode_payload,
    encode_payload,
    qr_matrix,
    render_marker,
)


def test_payload_round_trips():
    payload = encode_payload(47, "PT", 1250, 3000, out_edges=[(0, 52), (90, 61), (270, 33)])
    assert decode_payload(payload) == {
        "node_id": 47,
        "kind": "PT",
        "x_mm": 1250,
        "y_mm": 3000,
        "bearing_deg": 0,
        "out_edges": [(0, 52), (90, 61), (270, 33)],
    }


def test_payload_with_no_out_edges_round_trips():
    """A dead end is legal -- a charging bay need not lead anywhere."""
    assert decode_payload(encode_payload(9, "CH", 0, 0))["out_edges"] == []


def test_payload_stays_in_qr_alphanumeric_mode():
    """Byte mode would cost ~45% more modules, and modules are camera pixels."""
    segno = pytest.importorskip("segno")
    payload = encode_payload(999, "TR", 99999, 99999, out_edges=[(0, 1), (90, 2), (180, 3), (270, 4)])
    assert segno.make(payload, error="M").mode == "alphanumeric"


def test_unknown_kind_is_rejected():
    with pytest.raises(ValueError, match="unknown node kind"):
        encode_payload(1, "ZZ", 0, 0)


def test_negative_coordinates_are_rejected():
    """The payload has no sign, so the facility origin belongs at a corner."""
    with pytest.raises(ValueError, match="non-negative"):
        encode_payload(1, "PT", -100, 0)


def test_malformed_payloads_are_rejected():
    for bad in ("47.PT.1250", "47.XX.1.2.0.", "47.PT.1.2.0.0:"):
        with pytest.raises(ValueError):
            decode_payload(bad)


def test_bearings_wrap_to_one_turn():
    payload = encode_payload(1, "PT", 0, 0, out_edges=[(450, 2)])
    assert decode_payload(payload)["out_edges"] == [(90, 2)]


def test_render_places_modules_where_the_matrix_says():
    """Sample each module's centre and compare against the source matrix.

    Catches off-by-one scaling and origin errors without needing a decoder.
    """
    pytest.importorskip("PIL")
    payload = encode_payload(10, "PT", 1500, 1000, out_edges=[(0, 20)])
    spec = MarkerSpec()
    image = render_marker(payload, spec)
    pixels = image.load()

    matrix = qr_matrix(payload)
    modules = len(matrix)
    qr_px = spec.qr_mm * spec.pixels_per_mm
    scale = qr_px / modules
    origin = (spec.size_px - qr_px) / 2.0

    for row_index, row in enumerate(matrix):
        for col_index, bit in enumerate(row):
            x = int(origin + (col_index + 0.5) * scale)
            y = int(origin + (row_index + 0.5) * scale)
            dark = pixels[x, y][0] < 128
            assert dark == bool(bit), "module ({}, {}) wrong".format(row_index, col_index)


def test_border_is_the_trigger_colour_at_the_tile_edge():
    pytest.importorskip("PIL")
    spec = MarkerSpec()
    image = render_marker(encode_payload(10, "PT", 0, 0), spec)
    pixels = image.load()

    assert pixels[2, 2] == spec.border_rgb
    assert pixels[spec.size_px // 2, spec.size_px // 2] in ((0, 0, 0), (255, 255, 255))


def test_survives_the_camera_resolution():
    """Downsample the tile the way the robot's camera would, then check the
    modules still read correctly.

    This is the test that decides whether the optics geometry works at all. At
    the PROTO defaults the camera sees a 92.7 mm square across 256 px, so a
    60 mm / 33-module code lands near 5 px per module.
    """
    Image = pytest.importorskip("PIL.Image", reason="Pillow required")

    mast, wheel_radius, fov, resolution = 0.06, 0.02, 1.05, 256
    footprint_mm = 2 * (mast + wheel_radius) * math.tan(fov / 2) * 1000

    payload = encode_payload(10, "PT", 1500, 1000, out_edges=[(0, 20)])
    spec = MarkerSpec()
    tile = render_marker(payload, spec)

    # The camera sees `footprint_mm` of floor; crop the tile to that window,
    # centred, then resample to the sensor's pixel count.
    crop_px = int(footprint_mm * spec.pixels_per_mm)
    inset = (spec.size_px - crop_px) // 2
    window = tile.crop((inset, inset, inset + crop_px, inset + crop_px))
    frame = window.resize((resolution, resolution), Image.BILINEAR)
    pixels = frame.load()

    matrix = qr_matrix(payload)
    modules = len(matrix)
    qr_px_in_frame = spec.qr_mm / footprint_mm * resolution
    scale = qr_px_in_frame / modules
    origin = (resolution - qr_px_in_frame) / 2.0

    assert scale >= 4.0, "only {:.1f} px per module; decoding is unreliable".format(scale)

    wrong = 0
    for row_index, row in enumerate(matrix):
        for col_index, bit in enumerate(row):
            x = int(origin + (col_index + 0.5) * scale)
            y = int(origin + (row_index + 0.5) * scale)
            if (pixels[x, y][0] < 128) != bool(bit):
                wrong += 1

    assert wrong == 0, "{} of {} modules misread at camera resolution".format(
        wrong, modules * modules)
