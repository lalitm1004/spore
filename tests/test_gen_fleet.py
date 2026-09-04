import pytest

from tools.gen_fleet import compose_source, robot_configs, sensor_offsets, world_source

MANIFEST = {
    "track": {"plane_size": [4.0, 4.0], "track_size": [3.0, 2.0]},
    "robot": {"sensor_count": 3, "sensor_spacing": 0.02, "sensor_height": 0.015},
    "defaults": {
        "control": {"base_speed": 6.0, "pid": {"kp": 150.0, "ki": 0.0, "kd": 4.0}},
        "resources": {"memory": "256m", "cpus": "0.5"},
    },
    "robots": [
        {"name": "bot_01", "pose": {"x": 0.0, "y": -1.0, "theta": 0.0}},
        {
            "name": "bot_02",
            "pose": {"x": 0.3, "y": -1.0, "theta": 0.0},
            "control": {"pid": {"kp": 220.0}},
            "resources": {"memory": "128m"},
        },
    ],
}


def test_sensor_offsets_are_symmetric_and_left_to_right():
    assert sensor_offsets(count=3, spacing=0.02) == pytest.approx((0.02, 0.0, -0.02))
    assert sensor_offsets(count=2, spacing=0.02) == pytest.approx((0.01, -0.01))


def test_per_robot_config_merges_overrides_over_defaults():
    configs = {c["name"]: c for c in robot_configs(MANIFEST)}

    assert configs["bot_01"]["control"]["pid"]["kp"] == 150.0
    assert configs["bot_02"]["control"]["pid"]["kp"] == 220.0
    # The override must not drop the sibling gains.
    assert configs["bot_02"]["control"]["pid"]["kd"] == 4.0
    assert configs["bot_02"]["control"]["base_speed"] == 6.0


def test_per_robot_config_carries_the_sensor_geometry_from_the_robot_block():
    configs = robot_configs(MANIFEST)

    assert configs[0]["sensors"]["offsets"] == pytest.approx((0.02, 0.0, -0.02))


def test_duplicate_robot_names_are_rejected():
    manifest = dict(MANIFEST, robots=[{"name": "bot_01"}, {"name": "bot_01"}])

    with pytest.raises(ValueError, match="bot_01"):
        robot_configs(manifest)


def test_world_declares_every_robot_as_an_extern_controller():
    world = world_source(MANIFEST)

    for name in ("bot_01", "bot_02"):
        assert 'name "{}"'.format(name) in world
    assert world.count('controller "<extern>"') == 2


def test_compose_targets_each_robot_by_name_with_its_own_limits():
    compose = compose_source(MANIFEST)

    assert compose["services"]["bot_01"]["environment"]["ROBOT_NAME"] == "bot_01"
    assert compose["services"]["bot_01"]["mem_limit"] == "256m"
    assert compose["services"]["bot_02"]["mem_limit"] == "128m"
    assert compose["services"]["bot_02"]["cpus"] == "0.5"


def test_compose_services_run_as_the_host_user_so_telemetry_is_not_root_owned():
    compose = compose_source(MANIFEST)

    assert compose["services"]["bot_01"]["user"] == "${DOCKER_USER:-1000:1000}"


def test_compose_services_run_under_an_init_so_signals_reach_the_controller():
    compose = compose_source(MANIFEST)

    assert compose["services"]["bot_01"]["init"] is True


def test_compose_writes_telemetry_inside_the_mounted_project():
    # Only the repo is mounted, so a path outside it is not writable.
    environment = compose_source(MANIFEST)["services"]["bot_01"]["environment"]

    assert environment["TELEMETRY"] == "/project/out/bot_01.csv"
    assert environment["CONFIG"] == "/project/config/bot_01.yaml"


def test_world_and_compose_agree_on_the_robot_names():
    # The whole reason for generating both from one manifest.
    compose = compose_source(MANIFEST)
    world = world_source(MANIFEST)

    for name in compose["services"]:
        if name in ("sim", "supervisor"):
            continue
        assert 'name "{}"'.format(name) in world


import math

from tools.gen_fleet import TOP_DOWN_ORIENTATION, viewpoint_height


def test_viewpoint_is_top_down_looking_along_negative_z():
    world = world_source(MANIFEST)

    assert TOP_DOWN_ORIENTATION in world


def test_viewpoint_height_frames_the_whole_plane():
    # Identity orientation looks along +x, so a top-down view needs the
    # rotation above; the height must fit the plane in the field of view.
    height = viewpoint_height(plane_size=(4.0, 4.0), field_of_view=math.pi / 4)

    assert height >= 2.0 / math.tan(math.pi / 8)
    assert "position 0 0 {}".format(round(height, 3)) in world_source(MANIFEST)
