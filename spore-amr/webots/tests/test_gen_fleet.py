import pytest

from robot.config import ControlConfig
from tools.gen_fleet import (
    CONTROLLER_IMAGE, compose_source, robot_configs, sensor_offsets, world_source)

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
    # Two robots plus the supervisor, which is also launched externally.
    assert world.count('controller "<extern>"') == 3


def test_world_gives_every_robot_a_def_the_supervisor_can_resolve():
    """A supervisor's only handle on a PROTO instance is its DEF name."""
    world = world_source(MANIFEST)

    for name in ("bot_01", "bot_02"):
        assert "DEF {} LineBot".format(name.upper()) in world


def test_supervisor_is_unsynchronized():
    """A synchronized supervisor would stall the world whenever its own
    container is slow, which is the opposite of what a watcher should do."""
    world = world_source(MANIFEST)

    assert "supervisor TRUE" in world
    assert "synchronization FALSE" in world


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


def test_mission_duration_can_be_overridden_per_run():
    # CI smoke tests need a short run without regenerating the fleet.
    environment = compose_source(MANIFEST)["services"]["bot_01"]["environment"]

    assert environment["MISSION_DURATION"] == "${MISSION_DURATION:-120}"


def robot_services(compose):
    """The robot halves: firmware plus companion, one per robot in the world.

    Not the `-bot` containers beside them, and not the sim or the supervisor.
    """
    return {n: s for n, s in compose["services"].items()
            if n not in ("sim", "supervisor", "control") and not n.endswith("-bot")}


def test_compose_gives_every_robot_its_own_network_layer_bot():
    """The wiring that was missing for the entire life of this fleet.

    `compose.fleet.yml` is generated, and the generator could not run: its map
    source path was one directory too high after the repo was restructured, so
    the checked-in file went stale and kept a shape that predated the network
    layer entirely -- no identity, no peers, nothing to talk to. Every robot
    drove with nothing answering it. Nothing failed loudly, and nothing here
    would have noticed: `ROBOT_NAME` and `mem_limit` were the only things
    checked, and both were true the whole time the fleet was broken.

    So this asserts the wiring, for every robot rather than the first.
    """
    compose = compose_source(MANIFEST)
    robots = robot_services(compose)
    assert robots, "a fleet with no robots is not a fleet"

    for index, name in enumerate(robots):
        bot = "{}-bot".format(name)
        assert bot in compose["services"], "{} has no network layer".format(name)
        environment = compose["services"][bot]["environment"]

        # Identity: without it a bot cannot elect, be assigned to, or be found.
        assert environment["BOT_ID"] == str(index), name
        assert environment["OWN_ADDRESS"] == "{}:50051".format(bot), name
        assert "REGION_ID" in environment, name
        assert environment["WAREHOUSE_MAP"], name

        # One patience. The bot's hold ceiling and the firmware's junction
        # timeout must be the same number from the same place, or the fleet's
        # "every WAIT stays under the robot's patience" check guards a copy.
        firmware = (robot_configs(MANIFEST)[index].get("control") or {})
        expected = firmware.get("junction_timeout_s", ControlConfig().junction_timeout_s)
        assert float(environment["ROBOT_PATIENCE"]) == float(expected), name

        # Everyone else, so a bot with no leader yet has somewhere to ask.
        peers = environment["PEER_LEADERS"].split(",")
        assert "{}:50051".format(bot) not in peers, "{} lists itself".format(bot)
        assert len(peers) == len(robots) - 1, name

        # And the robot half has to be pointed at its own bot, not any other.
        assert compose["services"][name]["environment"]["NETWORK_ADDRESS"] == \
            "{}:50051".format(bot), name
        assert bot in compose["services"][name]["depends_on"], name


def test_the_network_layer_runs_on_an_image_that_can_actually_run_it():
    """The other half of why this fleet never worked.

    The Webots image is Ubuntu 22.04, so Python 3.10, and the planner needs
    3.11+ for `enum.StrEnum`. While the bot shared a container with the
    companion it raised ImportError on its first import, every time, silently.
    Moving the robot link off a unix socket is what allows the split -- a socket
    forces co-location and an address does not.
    """
    compose = compose_source(MANIFEST)
    for name in robot_services(compose):
        bot = compose["services"]["{}-bot".format(name)]
        assert bot["image"] != CONTROLLER_IMAGE, (
            "{}-bot is on the Webots image, which cannot import the network "
            "layer".format(name))


def test_no_robot_talks_to_a_fleet_wide_network_service():
    """One bot per robot is the architecture, not an implementation detail.

    `webots-implementation` wired every companion at a single `network:50051`
    holding the whole fleet's state. That is a coherent design and it is not
    this one -- see `spore-amr/network-layer/docs/boundary.md`. If it comes back
    it should come back deliberately, with that document rewritten, rather than
    by a merge nobody read closely.
    """
    compose = compose_source(MANIFEST)
    assert "network" not in compose["services"]

    addresses = {s["environment"]["NETWORK_ADDRESS"]
                 for s in robot_services(compose).values()}
    assert len(addresses) == len(robot_services(compose)), \
        "two robots share a network layer: {}".format(sorted(addresses))

    for service in compose["services"].values():
        assert not any("temp-network" in v for v in (service.get("volumes") or []))


def test_the_control_plane_can_reach_every_bot_and_nothing_else():
    """Orders enter through the control plane, which knows no leaders: it tries
    the bots it was told about until one accepts. Told about all of them, then,
    and only them -- a stale or missing address is an order that waits out five
    retries before landing, or never lands."""
    compose = compose_source(MANIFEST)
    control = compose["services"]["control"]
    addresses = set(control["environment"]["BOT_ADDRESSES"].split(","))
    assert addresses == {"{}-bot:50051".format(n) for n in robot_services(compose)}
    assert control["environment"]["WAREHOUSE_MAP"] == "/project/config/warehouse.json"


def test_world_and_compose_agree_on_the_robot_names():
    # The whole reason for generating both from one manifest.
    compose = compose_source(MANIFEST)
    world = world_source(MANIFEST)

    for name in robot_services(compose):
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


import pathlib

import yaml

from tools.gen_fleet import charging_spawns
from tools.manifest import TrackConfig

# A 4x4 lattice with three charging bays, enough to spawn a small fleet from.
GRAPH_MANIFEST = {
    "track": {
        "graph": {
            "rows": 4,
            "columns": 4,
            "spacing": 2.0,
            "kinds": [
                {"row": 0, "column": 0, "kind": "CH"},
                {"row": 0, "column": 3, "kind": "CH"},
                {"row": 3, "column": 0, "kind": "CH"},
            ],
        },
        "line_width": 0.02,
    },
    "robot": {"sensor_count": 3, "sensor_spacing": 0.02, "sensor_height": 0.015},
    "defaults": {"control": {"base_speed": 6.0}, "resources": {"cpus": "0.5"}},
    "robots": {"count": 3, "spawn": "charging", "start_interval_s": 4.0},
}


def charging_nodes(manifest):
    graph = TrackConfig.from_dict(manifest["track"]).build_graph()
    return {n.node_id: n for n in graph.nodes.values() if n.kind == "CH"}


def test_robots_start_on_the_charging_node_itself():
    """The START node is the bay, not a point part-way down its lane. A robot
    centred on the node still starts clear of the 100 mm tile, because the
    colour sensor sits 125 mm forward of the wheel axle."""
    bays = charging_nodes(GRAPH_MANIFEST)

    for pose in charging_spawns(GRAPH_MANIFEST, count=3):
        bay = bays[pose["from_node"]]
        assert pose["x"] == pytest.approx(bay.x)
        assert pose["y"] == pytest.approx(bay.y)


def test_each_robot_starts_on_a_bay_of_its_own():
    poses = charging_spawns(GRAPH_MANIFEST, count=3)

    assert len({p["from_node"] for p in poses}) == 3


def test_robots_face_out_along_a_lane_leaving_their_bay():
    """Facing into open lane, not into the bay: the first thing a released
    robot does is drive, and it must have a line under it to follow."""
    graph = TrackConfig.from_dict(GRAPH_MANIFEST["track"]).build_graph()

    for pose in charging_spawns(GRAPH_MANIFEST, count=3):
        bearings = [graph.bearing(pose["from_node"], n)
                    for n in graph.neighbours(pose["from_node"])]
        # Poses are written to 4 dp, as the world file wants them.
        assert any(abs(pose["theta"] - b) < 1e-4 for b in bearings)


def test_start_delays_are_staggered_by_the_manifest_interval():
    """Sequential release: one robot leaves every interval, so paired bays
    never reach their shared junction at the same instant."""
    configs = robot_configs(GRAPH_MANIFEST)

    delays = [c["control"]["start_delay_s"] for c in configs]
    assert delays == [0.0, 4.0, 8.0]


def test_without_an_interval_the_whole_fleet_starts_together():
    """The stagger is opt-in, so existing manifests behave as they did."""
    manifest = dict(GRAPH_MANIFEST,
                    robots={"count": 3, "spawn": "charging"})

    assert [c["control"]["start_delay_s"] for c in robot_configs(manifest)] == [0.0] * 3


def test_bay_mates_are_released_far_apart_in_the_sequence():
    """Two bays sharing a junction are 2 m of spur from it -- about 16.7 s at
    cruise. Releasing them back to back would put both into that junction at
    once, which is the deadlock the stagger exists to prevent, so they must be
    a full pass through the junctions apart."""
    manifest = yaml.safe_load(pathlib.Path("fleet.yaml").read_text())
    graph = TrackConfig.from_dict(manifest["track"]).build_graph()

    order = [p["from_node"] for p in charging_spawns(manifest, count=10)]
    junctions = [graph.neighbours(node)[0] for node in order]

    # Every junction is visited once before any is visited twice.
    assert len(set(junctions[:5])) == 5
    for slot, node in enumerate(order):
        mate = junctions.index(junctions[slot])
        assert mate == slot or abs(mate - slot) >= 5


def test_each_robot_is_told_the_heading_it_was_placed_on():
    """The world file and the config must agree: a `TURN` is an absolute
    bearing, and the firmware's only frame of reference is this number."""
    poses = charging_spawns(GRAPH_MANIFEST, count=3)
    configs = robot_configs(GRAPH_MANIFEST)

    for pose, config in zip(poses, configs):
        assert config["odometry"]["start_theta"] == pytest.approx(pose["theta"])
