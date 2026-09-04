import pathlib

import pytest
import yaml

from tools.manifest import TrackConfig
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


def test_the_shipped_manifest_produces_a_power_of_two_texture():
    # Webots silently rescales non-power-of-two textures, which resamples the
    # line edges that the IR sensors read.
    manifest = yaml.safe_load(pathlib.Path("fleet.yaml").read_text())
    config = TrackConfig.from_dict(manifest["track"])
    spec = TrackImageSpec(size=config.plane_size, pixels_per_metre=config.pixels_per_metre)

    for dimension in (spec.width_px, spec.height_px):
        assert dimension & (dimension - 1) == 0, "{} is not a power of two".format(dimension)
