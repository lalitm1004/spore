import pytest

from tools.manifest import TrackConfig


def test_track_config_applies_defaults():
    config = TrackConfig.from_dict({"size": [4.0, 4.0]})

    assert config.size == (4.0, 4.0)
    assert config.shape == "oval"
    assert config.margin == 0.5
    assert config.line_width == 0.02
    assert config.pixels_per_metre == 512


def test_track_config_builds_a_centerline_inset_by_the_margin():
    config = TrackConfig.from_dict({"size": [4.0, 3.0], "margin": 0.5})

    centerline = config.build_centerline()

    assert (centerline.width, centerline.height) == (3.0, 2.0)


def test_unknown_shape_is_rejected_by_name():
    config = TrackConfig.from_dict({"size": [4.0, 4.0], "shape": "hexagram"})

    with pytest.raises(ValueError, match="hexagram"):
        config.build_centerline()


def test_margin_larger_than_the_plane_is_rejected():
    config = TrackConfig.from_dict({"size": [4.0, 3.0], "margin": 1.6})

    with pytest.raises(ValueError, match="margin"):
        config.build_centerline()
