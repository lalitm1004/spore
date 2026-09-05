import pathlib

import pytest
import yaml

from tools.manifest import MarkerConfig, TrackConfig
from tools.track.raster import TrackImageSpec


def test_track_config_applies_defaults():
    config = TrackConfig.from_dict({"plane_size": [4.0, 4.0], "track_size": [3.0, 2.0]})

    assert config.plane_size == (4.0, 4.0)
    assert config.track_size == (3.0, 2.0)
    assert config.shape == "oval"
    assert config.line_width == 0.02
    assert config.pixels_per_metre == 512


def test_track_config_builds_the_centerline_at_the_requested_track_size():
    config = TrackConfig.from_dict({"plane_size": [4.0, 4.0], "track_size": [3.0, 2.0]})

    centerline = config.build_centerline()

    assert (centerline.width, centerline.height) == (3.0, 2.0)


def test_unknown_shape_is_rejected_by_name():
    config = TrackConfig.from_dict(
        {"plane_size": [4.0, 4.0], "track_size": [3.0, 2.0], "shape": "hexagram"}
    )

    with pytest.raises(ValueError, match="hexagram"):
        config.build_centerline()


def test_a_track_wider_than_its_plane_is_rejected():
    config = TrackConfig.from_dict({"plane_size": [4.0, 4.0], "track_size": [4.5, 2.0]})

    with pytest.raises(ValueError, match="does not fit"):
        config.build_centerline()


def test_a_track_whose_line_would_touch_the_plane_edge_is_rejected():
    # Half the line width must still land inside the plane.
    config = TrackConfig.from_dict(
        {"plane_size": [4.0, 4.0], "track_size": [4.0, 2.0], "line_width": 0.02}
    )

    with pytest.raises(ValueError, match="does not fit"):
        config.build_centerline()


def test_the_marker_tile_is_a_power_of_two():
    """Webots silently rescales a non-power-of-two texture, and rescaling a QR
    resamples the very module edges the decoder reads.

    This used to be asserted of the floor as well, because the floor carried
    the guide line and the IR array read it. The lanes are geometry now -- see
    `tools/gen_fleet.lane_source` -- so the floor is the warehouse drawing and
    nothing senses it. Rescaling a picture costs nothing. Rescaling a QR code
    costs the read.
    """
    manifest = yaml.safe_load(pathlib.Path("fleet.yaml").read_text())
    spec = MarkerConfig.from_dict(manifest.get("markers")).spec
    side = round(spec.tile_mm * spec.pixels_per_mm)

    assert side & (side - 1) == 0, "{} px is not a power of two".format(side)


def test_the_floor_extends_beyond_the_outermost_nodes():
    """A boundary node with no floor past it is a robot that runs out of
    world. Off the plane every IR sensor reads black_ref, which the estimator
    renders as a perfectly centred line at maximum confidence -- so the robot
    does not stop, it drives straight for ever. One went 18 m."""
    manifest = yaml.safe_load(pathlib.Path("fleet.yaml").read_text())
    config = TrackConfig.from_dict(manifest["track"])

    assert config.plane_size[0] > config.track_size[0]
    assert config.plane_size[1] > config.track_size[1]
