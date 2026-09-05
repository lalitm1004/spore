"""Generate the world, the compose file, and per-robot configs from fleet.yaml.

One manifest is the source of truth, so a robot's name in the world can never
drift from the `--robot-name` its container connects with.
"""

import argparse
import math
import pathlib
from typing import List, Tuple

import yaml

from tools.manifest import MarkerConfig, TrackConfig, deep_merge

# Identity orientation looks along +x with +z up. This rotation maps x -> -z and
# z -> +y, so the camera looks straight down with world +y up in the image,
# matching how the track texture is laid out.
TOP_DOWN_ORIENTATION = "orientation -0.5774 0.5774 0.5774 2.0944"
FIELD_OF_VIEW = math.pi / 4
VIEW_MARGIN = 1.15

BOT_IMAGE = "amr-bot:dev"
CONTROLLER_IMAGE = "sih2026/controller:dev"
SIM_IMAGE = "sih2026/sim:dev"


def sensor_offsets(count: int, spacing: float) -> Tuple[float, ...]:
    """Lateral sensor positions, left to right, in the robot frame (+y left)."""
    first = (count - 1) / 2 * spacing
    return tuple(first - i * spacing for i in range(count))


def viewpoint_height(plane_size, field_of_view: float = FIELD_OF_VIEW) -> float:
    """Camera height that fits the whole ground plane in view."""
    half = max(plane_size) / 2
    return half / math.tan(field_of_view / 2) * VIEW_MARGIN


def charging_spawns(manifest: dict, count: int) -> List[dict]:
    """Poses at the charging nodes themselves, one robot per bay.

    A robot sits on its bay facing out along the lane leaving it -- the START
    node is the bay, not a point part-way down its spur. Sitting on the node
    does not trip the colour trigger: the tile is 100 mm long and centred on
    the node, so its far edge is 50 mm out, while the colour sensor is mounted
    125 mm forward of the wheel axle. The robot starts past the tile and drives
    away from it.

    Bays are degree-1 spurs that come in facing pairs sharing one junction, so
    two robots leaving paired bays would reach that junction at the same
    instant and deadlock. That is handled by releasing the fleet one robot at a
    time (`robots.start_interval_s`), not by spreading the poses out.
    """
    track = TrackConfig.from_dict(manifest["track"])
    graph = track.build_graph()

    bays = sorted((n for n in graph.nodes.values() if n.kind == "CH"),
                  key=lambda n: n.node_id)
    if not bays:
        raise ValueError("no CH nodes in this track to spawn at")

    # Order the bays so that bay-mates are as far apart in the release
    # sequence as the fleet allows: every junction's first bay goes out, then
    # every junction's second. On the real window that is 5 junctions of 2
    # bays, so a pair is 5 slots apart -- 20 s at a 4 s interval, against the
    # 16.7 s a robot needs to cover its 2 m spur. The pair never meets.
    by_junction = {}
    for bay in bays:
        neighbours = graph.neighbours(bay.node_id)
        if not neighbours:
            raise ValueError(
                "charging bay {} has no lane leaving it".format(bay.node_id))
        by_junction.setdefault(neighbours[0], []).append(bay)

    ordered = []
    for rank in range(max(len(g) for g in by_junction.values())):
        for _, group in sorted(by_junction.items()):
            if rank < len(group):
                ordered.append(group[rank])

    poses = []
    for index in range(count):
        bay = ordered[index % len(ordered)]
        neighbours = graph.neighbours(bay.node_id)
        poses.append({
            "x": round(bay.x, 3),
            "y": round(bay.y, 3),
            "theta": round(graph.bearing(bay.node_id, neighbours[0]), 4),
            "from_node": bay.node_id,
        })
    return poses


def robot_configs(manifest: dict) -> List[dict]:
    defaults = manifest.get("defaults") or {}
    robot_block = manifest.get("robot") or {}
    offsets = sensor_offsets(
        count=robot_block.get("sensor_count", 3),
        spacing=robot_block.get("sensor_spacing", 0.02),
    )

    # `robots: {count: N, spawn: charging}` places the fleet from the track
    # itself, so a layout change cannot leave the poses behind.
    entries = manifest["robots"]
    if isinstance(entries, dict):
        count = int(entries.get("count", 1))
        poses = charging_spawns(manifest, count)
        # `start_interval_s` releases the fleet one robot at a time. Bays are
        # degree-1 spurs pairing onto one junction, so a simultaneous start
        # puts two robots into that junction on the same tick.
        interval = float(entries.get("start_interval_s", 0.0) or 0.0)
        entries = [
            {
                "name": "bot_{:02d}".format(i + 1),
                "pose": poses[i],
                "control": {"start_delay_s": round(i * interval, 3)},
                # The firmware has no compass, so the frame its turns are
                # absolute in has to be handed to it. Same number the world
                # file places the robot on -- they cannot disagree.
                "odometry": {"start_theta": poses[i]["theta"]},
            }
            for i in range(min(count, len(poses)))
        ]

    configs = []
    seen = set()
    for entry in entries:
        name = entry["name"]
        if name in seen:
            raise ValueError("duplicate robot name {!r} in the manifest".format(name))
        seen.add(name)

        overrides = {k: v for k, v in entry.items() if k not in ("name", "pose")}
        merged = deep_merge(defaults, overrides)
        merged["name"] = name
        merged["pose"] = entry.get("pose") or {"x": 0.0, "y": 0.0, "theta": 0.0}
        merged.setdefault("sensors", {})
        merged["sensors"] = deep_merge({"offsets": offsets}, merged["sensors"])
        configs.append(merged)

    return configs


# Markers sit a hair above the ground plane. Coplanar faces z-fight and the
# texture flickers between the two, which the colour sensor reads as noise.
MARKER_LIFT = 0.001

# Set by main() once the floor is rendered, so world_source names the same file
# the browser will fetch. The streaming viewer caches the floor by URL, so a
# regenerated track kept arriving in the browser as the previous one -- world
# right, file on disk right, picture stale.
TRACK_TEXTURE = ["track.png"]

# The largest texture Webots will actually read here. A single 16384x8192 floor
# for the whole warehouse failed with "Unable to read image data" -- the world
# loaded, the ground had no texture at all, and every robot sat reading blank
# white floor and reporting a lost line. 8192x4096 is the size the 32 x 16 m
# window has always used, so it is known good.
#
# The fix is more planes, not a coarser floor. Resolution is the one thing that
# cannot be given up: the IR array reads the rendered floor, and a 20 mm lane
# at 256 px/m is 5 px. Halving that to fit a bigger map on one texture would
# quietly wreck line following everywhere.
MAX_TEXTURE_PX = 8192


# The lane ink, matching what the raster floor used to draw. Normalised: the
# renderer wants 0-1, the rasteriser wanted 0-255.
LANE_INK = (43 / 255.0, 52 / 255.0, 57 / 255.0)

# Lanes sit above the floor and below the marker tiles, so a tile still covers
# the line it crosses. Both clear the ground plane, which would otherwise
# z-fight and flicker -- and the IR array reads that flicker as noise.
LANE_LIFT = 0.0005

LANE_TEMPLATE = """    Pose {{
      translation {x:.4f} {y:.4f} 0
      rotation 0 0 1 {heading:.5f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {r:.4f} {g:.4f} {b:.4f}
            roughness 1
            metalness 0
          }}
          geometry Plane {{
            size {length:.4f} {width:.4f}
          }}
        }}
      ]
    }}
"""


def lane_source(track, graph):
    """The lanes as geometry rather than as pixels.

    This is the whole floor budget. A 20 mm line has to be a few pixels wide
    for the IR array to find its centre, which fixes the floor's resolution at
    256 px/m -- and over the full 128 x 64 m warehouse that is a 32768 x 16384
    texture: 2.1 GB of memory to express 952 straight lines, almost all of it
    white. Webots would not even load a quarter of it ("Unable to read image
    data"), and the robots sat on an untextured floor reading blank white.

    Geometry costs nothing and is *sharper*: a plane's edge is exact, where a
    raster line is quantised to whole pixels and then mipmapped. The sensors do
    not care which it is -- an infra-red DistanceSensor reads the surface it
    hits, and the marker tiles have always been lifted planes exactly like
    these.

    Safe to lift, too: the sensor lookup table is flat from 0 to its mounting
    height, so half a millimetre of clearance changes the reading by nothing.

    One Solid holding many Poses rather than one Solid per lane -- 952 Solids
    is 952 lots of physics bookkeeping for something that never moves.
    """
    blocks = []
    for edge in graph.edges:
        a, b = graph.nodes[edge.a], graph.nodes[edge.b]
        blocks.append(LANE_TEMPLATE.format(
            x=(a.x + b.x) / 2.0, y=(a.y + b.y) / 2.0,
            heading=graph.bearing(edge.a, edge.b),
            length=graph.length(edge.a, edge.b), width=track.line_width,
            r=LANE_INK[0], g=LANE_INK[1], b=LANE_INK[2]))

    # A square at each node fills the notch where two lanes meet at an angle.
    # The rasteriser drew a dot here for the same reason.
    for node in graph.nodes.values():
        blocks.append(LANE_TEMPLATE.format(
            x=node.x, y=node.y, heading=0.0,
            length=track.line_width, width=track.line_width,
            r=LANE_INK[0], g=LANE_INK[1], b=LANE_INK[2]))

    return """DEF LANES Solid {{
  translation 0 0 {lift}
  children [
{children}  ]
  name "lanes"
}}
""".format(lift=LANE_LIFT, children="".join(blocks))


GROUND_TEMPLATE = """DEF GROUND_{row}_{column} Solid {{
  translation {cx:.4f} {cy:.4f} 0
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 1 1
        roughness 0
        metalness 0
        baseColorMap ImageTexture {{
          url [
            "../textures/{texture}"
          ]
        }}
      }}
      geometry Plane {{
        size {size_x} {size_y}
      }}
    }}
  ]
  name "ground_{row}_{column}"
  boundingObject Plane {{
    size {size_x} {size_y}
  }}
}}
"""


def ground_source(track):
    """One Solid per floor tile.

    A `boundingObject Plane` with an explicit size on each, rather than the
    unbounded default: an infinite collision plane per tile would have every
    tile colliding across the whole world.
    """
    blocks = []
    for tile, texture in zip(floor_tiles(track), TRACK_TEXTURE):
        blocks.append(GROUND_TEMPLATE.format(
            row=tile["row"], column=tile["column"],
            cx=tile["centre"][0], cy=tile["centre"][1],
            size_x=tile["size_m"][0], size_y=tile["size_m"][1],
            texture=texture))
    return "".join(blocks)


def floor_tiles(track):
    """The floor as a grid of planes, each within `MAX_TEXTURE_PX`.

    Returns (row, column, origin_cm, size_m, centre_xy) per tile. One tile for
    anything that already fits, so a small window generates exactly the world
    it always did.
    """
    width_m, height_m = track.plane_size
    ppm = track.pixels_per_metre
    columns = max(1, math.ceil(width_m * ppm / MAX_TEXTURE_PX))
    rows = max(1, math.ceil(height_m * ppm / MAX_TEXTURE_PX))

    tile_w, tile_h = width_m / columns, height_m / rows
    # The plane's corner, not the window's: the floor extends a margin beyond
    # the graph so a robot overshooting a boundary node still has floor.
    origin_x, origin_y = (track.warehouse.plane_origin_cm if track.warehouse
                          else (0.0, 0.0))

    tiles = []
    for row in range(rows):
        for column in range(columns):
            tiles.append({
                "row": row,
                "column": column,
                # The source window this tile covers, in the same centimetres
                # the whole-floor window uses -- the renderer crops to it.
                "origin_cm": (origin_x + column * tile_w * 100.0,
                              origin_y + row * tile_h * 100.0),
                "size_m": (tile_w, tile_h),
                # Its centre on the plane, which is centred on the world origin.
                "centre": (-width_m / 2.0 + (column + 0.5) * tile_w,
                           -height_m / 2.0 + (row + 0.5) * tile_h),
            })
    return tiles


def graph_marker_source(manifest: dict, graph) -> str:
    """One marker Solid per graph node.

    Markers sit at nodes rather than at arclengths along a curve, because a
    node is where a robot has a decision to make -- which is the only place a
    marker earns the 155 mm of blind crossing it costs.
    """
    track = TrackConfig.from_dict(manifest["track"])
    markers = MarkerConfig.from_dict(manifest.get("markers"))
    side = markers.spec.tile_mm / 1000.0
    plane = track.plane_size[0]

    blocks = []
    for node in sorted(graph.nodes.values(), key=lambda n: n.node_id):
        # Align the tile with the first lane leaving the node, so it is square
        # to at least one approach rather than to the world axes.
        neighbours = graph.neighbours(node.node_id)
        heading = graph.bearing(node.node_id, neighbours[0]) if neighbours else 0.0
        blocks.append(MARKER_TEMPLATE.format(
            node_id=node.node_id, x=node.x, y=node.y,
            lift=MARKER_LIFT, heading=heading, side=side))
    return "".join(blocks)


MARKER_TEMPLATE = '''DEF MARKER_{node_id} Solid {{
  translation {x:.4f} {y:.4f} {lift}
  rotation 0 0 1 {heading:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 1 1
        roughness 1
        metalness 0
        baseColorMap ImageTexture {{
          url [
            "../textures/markers/node_{node_id:03d}.png"
          ]
        }}
      }}
      geometry Plane {{
        size {side} {side}
      }}
    }}
  ]
  name "marker_{node_id:03d}"
}}
'''


def marker_source(manifest: dict) -> str:
    """Solids for every floor marker, each carrying its own texture.

    A marker is a separate small plane rather than pixels in the track texture:
    at the track's 512 px/m a QR module would be one texel wide, and Webots
    would mipmap the finder patterns away. Its own tile decouples marker
    resolution from track resolution.
    """
    track = TrackConfig.from_dict(manifest["track"])
    markers = MarkerConfig.from_dict(manifest.get("markers"))
    if not markers.nodes:
        return ""

    centerline = track.build_centerline()
    side = markers.spec.tile_mm / 1000.0

    blocks = []
    for node in markers.nodes:
        x, y, heading = node.world_pose(centerline)
        blocks.append('''DEF MARKER_{node_id} Solid {{
  translation {x:.4f} {y:.4f} {lift}
  rotation 0 0 1 {heading:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 1 1
        roughness 1
        metalness 0
        baseColorMap ImageTexture {{
          url [
            "../textures/markers/node_{node_id:03d}.png"
          ]
        }}
      }}
      geometry Plane {{
        size {side} {side}
      }}
    }}
  ]
  name "marker_{node_id:03d}"
}}
'''.format(node_id=node.node_id, x=x, y=y, lift=MARKER_LIFT, heading=heading, side=side))

    return "".join(blocks)


def obstacle_source(manifest: dict) -> str:
    """Boxes on the floor for the obstacle reflex to find.

    Unlike marker tiles these carry a `boundingObject`: a lidar ray has to hit
    something, and a marker deliberately has nothing to hit so the IR array
    can see the lane texture through it.
    """
    blocks = []
    for index, entry in enumerate(manifest.get("obstacles") or []):
        size = entry.get("size") or [0.12, 0.12, 0.12]
        blocks.append('''DEF OBSTACLE_{index} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.85 0.15 0.1
        roughness 0.6
        metalness 0
      }}
      geometry Box {{
        size {sx} {sy} {sz}
      }}
    }}
  ]
  name "obstacle_{index}"
  boundingObject Box {{
    size {sx} {sy} {sz}
  }}
}}
'''.format(index=index, x=entry["x"], y=entry["y"], z=size[2] / 2.0,
           sx=size[0], sy=size[1], sz=size[2]))
    return "".join(blocks)


def world_source(manifest: dict) -> str:
    track = TrackConfig.from_dict(manifest["track"])
    robot_block = manifest.get("robot") or {}
    plane_x, plane_y = track.plane_size

    header = '''#VRML_SIM R2025a utf8

EXTERNPROTO "../protos/LineBot.proto"

WorldInfo {{
  info [
    "Generated by tools/gen_fleet.py -- edit fleet.yaml, not this file."
  ]
  basicTimeStep 16
  contactProperties [
    ContactProperties {{
      material1 "slippery"
      coulombFriction [
        0
      ]
    }}
  ]
}}
Viewpoint {{
  {orientation}
  position 0 0 {height}
}}
Background {{
  skyColor [
    0.6 0.7 0.85
  ]
}}
DirectionalLight {{
  direction 0.3 0.4 -1
  intensity 2.5
  castShadows FALSE
}}
{ground}{lanes}'''.format(plane_x=plane_x, plane_y=plane_y,
           orientation=TOP_DOWN_ORIENTATION,
           ground=ground_source(track),
           lanes=lane_source(track, track.build_graph()) if track.is_graph else "",
           height=round(viewpoint_height(track.plane_size), 3))

    if track.is_graph:
        body = [graph_marker_source(manifest, track.build_graph()),
                obstacle_source(manifest)]
    else:
        body = [marker_source(manifest), obstacle_source(manifest)]
    for config in robot_configs(manifest):
        pose = config["pose"]
        body.append('''DEF {def_name} LineBot {{
  translation {x} {y} 0.02
  rotation 0 0 1 {theta}
  name "{name}"
  controller "<extern>"
  sensorCount {count}
  sensorSpacing {spacing}
  sensorHeight {height}
  bodyLength {body_l}
  bodyWidth {body_w}
  bodyHeight {body_h}
}}
'''.format(
            def_name=config["name"].upper(),
            x=pose.get("x", 0.0),
            y=pose.get("y", 0.0),
            theta=pose.get("theta", 0.0),
            name=config["name"],
            count=robot_block.get("sensor_count", 3),
            spacing=robot_block.get("sensor_spacing", 0.02),
            height=robot_block.get("sensor_height", 0.015),
            body_l=robot_block.get("body_length", 0.50),
            body_w=robot_block.get("body_width", 0.50),
            body_h=robot_block.get("body_height", 0.10),
        ))

    body.append("""DEF SUPERVISOR Robot {
  name "supervisor"
  controller "<extern>"
  supervisor TRUE
  synchronization FALSE
}
""")

    return header + "".join(body)


# Mirrors the defaults in LineBot.proto. The firmware cannot ask Webots where
# its own sensors are mounted -- on hardware it could not either -- so the
# geometry is written into each config from the same manifest that builds the
# world, and the two cannot drift apart.
PROTO_OPTICS = {
    "color_sensor_x": 0.325,
    "camera_x": 0.295,
    "ir_array_x": 0.07,
    "camera_mast": 0.060,
    "camera_fov": 1.05,
    "camera_resolution": 512,
}
WHEEL_RADIUS = 0.02

# Calibrated against ground truth, not read off the model: see
# robot.config.OdometryConfig for the measurement.
ODOMETRY = {"wheel_radius": 0.02, "track_width": 0.0994}


def optics_config(manifest: dict) -> dict:
    """Optics geometry for the firmware, derived once from the manifest."""
    robot_block = manifest.get("robot") or {}
    overrides = robot_block.get("optics") or {}
    geometry = deep_merge(PROTO_OPTICS, overrides)
    markers = MarkerConfig.from_dict(manifest.get("markers"))
    track = TrackConfig.from_dict(manifest["track"])
    # A graph track puts a marker at every node, so its markers do not appear
    # in the manifest's `markers.nodes` -- only the tile geometry does.
    # Deriving "are there markers" from that list alone silently disabled the
    # optics on every graph world.
    has_markers = bool(markers.nodes) or track.is_graph

    height = geometry["camera_mast"] + WHEEL_RADIUS
    footprint = 2 * height * math.tan(geometry["camera_fov"] / 2)

    return {
        "enabled": has_markers,
        "color_sensor_x": geometry["color_sensor_x"],
        "ir_array_x": geometry["ir_array_x"],
        "camera_x": geometry["camera_x"],
        "camera_footprint": round(footprint, 5),
        "tile_length": markers.spec.tile_mm / 1000.0,
        "code_size": markers.spec.qr_mm / 1000.0,
        "border_rgb": list(markers.spec.border_rgb),
        "border_tolerance": 0.30,
    }


def spawn_region_ids(manifest: dict, configs: List[dict]) -> dict:
    """Which region each robot starts in, read off the track it spawns on.

    Taken from the map rather than configured, for the same reason the poses
    are: a layout change must not be able to leave a stale region behind.
    """
    spawns = {c["name"]: (c.get("pose") or {}).get("from_node") for c in configs}
    if not any(node_id is not None for node_id in spawns.values()):
        # An oval or any other track with no lane graph: there are no regions to
        # be in, so every bot boots in region 0 and migrates once it scans a QR
        # code that says otherwise.
        return {name: 0 for name in spawns}

    track = TrackConfig.from_dict(manifest["track"])
    graph = track.build_graph()
    return {
        name: getattr(graph.nodes.get(node_id), "region_id", 0) or 0
        for name, node_id in spawns.items()
    }


def compose_source(manifest: dict) -> dict:
    services = {}
    configs = robot_configs(manifest)
    robot_names = [c["name"] for c in configs]
    spawn_regions = spawn_region_ids(manifest, configs)
    for index, config in enumerate(configs):
        name = config["name"]
        resources = config.get("resources") or {}
        # Two containers per robot, and the split is not arbitrary.
        #
        # The *robot* half -- firmware and companion -- runs on the Webots
        # image, which is Ubuntu 22.04 and therefore Python 3.10. The network
        # layer needs 3.11+, so for as long as the two shared a container the
        # bot raised ImportError on startup and the fleet ran with nothing
        # answering it. A unix socket forced them together; an address does not.
        #
        # So each robot's bot gets the image built for it. It is the thing that
        # elects leaders, takes jobs, reserves nodes and answers this robot's
        # routing questions, and it needs a fleet identity, its peers' addresses
        # for bootstrap discovery, and the map (PROTOCOL.md §16). One per robot,
        # not one for the fleet: spore-amr/network-layer/docs/boundary.md.
        bot_name = "{}-bot".format(name)
        services[bot_name] = {
            "image": BOT_IMAGE,
            "build": {"context": "../network-layer"},
            "working_dir": "/app",
            "init": True,
            "environment": {
                "BOT_ID": str(index),
                # The region the robot spawns in. It migrates on its own once
                # it scans a QR code somewhere else, so this only has to be
                # right at boot.
                "REGION_ID": str(spawn_regions.get(name, 0)),
                "OWN_ADDRESS": "{}:50051".format(bot_name),
                "PEER_LEADERS": ",".join(
                    "{}-bot:50051".format(other)
                    for other in robot_names if other != name),
                "WAREHOUSE_MAP": "/project/config/warehouse.json",
                "GRPC_HOST": "0.0.0.0",
                "GRPC_PORT": "50051",
                # Read-only introspection, so `fleet.sh` can show what the
                # coordination layer is actually doing -- who leads, who is
                # where, what is claimed. This is a local demo fleet, the same
                # call `network-layer/up.py` makes for its test fleets. Leave it
                # off in production; there is nothing to write through it now
                # that the injection RPCs are gone, but it still exposes state.
                "ADMIN_ENABLED": "1",
            },
            "volumes": ["./:/project:ro"],
            "mem_limit": "192m",
            "cpus": "0.5",
        }

        services[name] = {
            "image": CONTROLLER_IMAGE,
            "depends_on": ["sim", bot_name],
            # The network layer is a sibling of this project, so it is outside
            # the `./:/project` mount and needs its own. Mounted rather than
            # copied: the companion imports the wire contract from it, and a
            # copied client is a client that drifts.
            "volumes": ["./:/project", "../network-layer:/network-layer:ro"],
            "working_dir": "/project",
            "user": "${DOCKER_USER:-1000:1000}",
            "init": True,
            "environment": {
                "ROBOT_NAME": name,
                "CONFIG": "/project/config/{}.yaml".format(name),
                "TELEMETRY": "/project/out/{}.csv".format(name),
                "SIM_HOST": "sim",
                "MISSION_DURATION": "${{MISSION_DURATION:-{}}}".format(
                    config.get("mission_duration_s", 120)),
                # This robot's own bot, by name. Not a fleet-wide service.
                "NETWORK_ADDRESS": "{}:50051".format(bot_name),
                "WAREHOUSE_MAP": "/project/config/warehouse.json",
            },
            "mem_limit": resources.get("memory", "256m"),
            "cpus": resources.get("cpus", "0.5"),
        }

    services["supervisor"] = {
        "image": CONTROLLER_IMAGE,
        "depends_on": ["sim"],
        "volumes": ["./:/project"],
        "working_dir": "/project",
        "user": "${DOCKER_USER:-1000:1000}",
        "init": True,
        "environment": {
            "ROBOT_NAME": "supervisor",
            "ROLE": "supervisor",
            "SIM_HOST": "sim",
            "MISSION_DURATION": "${MISSION_DURATION:-120}",
        },
        "mem_limit": "256m",
        "cpus": "0.25",
    }

    return {"services": services}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=pathlib.Path, default=pathlib.Path("fleet.yaml"))
    parser.add_argument("--skip-markers", action="store_true",
                        help="do not re-render marker textures")
    parser.add_argument("--window", metavar="X_CM,Y_CM,W_M,H_M", default=None,
                        help="cut a smaller piece of the same warehouse")
    args = parser.parse_args(argv)

    manifest = yaml.safe_load(args.manifest.read_text())

    # A chunk of the same warehouse, for when the whole one is too much to
    # watch. The full map is 881 marker tiles -- 924 MB of texture, which a
    # browser rendering the w3d stream cannot expand -- so the streaming
    # viewer only works on a piece of it. Everything else still comes from the
    # manifest; this overrides the window and nothing more, so `fleet.yaml`
    # stays the source of truth for the fleet itself.
    if args.window:
        origin_x, origin_y, width_m, height_m = (
            float(v) for v in args.window.split(","))
        manifest["track"]["warehouse"]["origin_cm"] = [origin_x, origin_y]
        manifest["track"]["warehouse"]["size_m"] = [width_m, height_m]
        print("window override: ({:.0f}, {:.0f}) cm, {:.0f} x {:.0f} m".format(
            origin_x, origin_y, width_m, height_m))

    # Markers are generated here rather than as a separate step: a world that
    # references a texture nobody rendered is a world that loads with white
    # squares where its codes should be, and nothing says so until a robot
    # drives over one and reads nothing.
    track = TrackConfig.from_dict(manifest["track"])

    if track.is_graph:
        # The graph is the source for everything: the floor texture, the
        # markers, and the map the robots read to know where lanes lead.
        import json

        from tools.track.raster import TrackImageSpec, render_graph
        from tools.track.warehouse import to_document

        graph = track.build_graph()
        spec = TrackImageSpec(size=track.plane_size,
                              pixels_per_metre=track.pixels_per_metre)
        pathlib.Path("textures").mkdir(exist_ok=True)

        import hashlib

        tiles = floor_tiles(track)
        for stale in pathlib.Path("textures").glob("track-*.png"):
            stale.unlink()
        del TRACK_TEXTURE[:]

        for tile in tiles:
            if track.warehouse is not None:
                # Use the layout tool's own drawing as the floor, so the
                # simulated warehouse looks like the warehouse rather than
                # like something this project drew from the same data. The
                # guide line is redrawn on top at its true width -- the map's
                # hairlines are a diagram.
                #
                # Rendered per tile, cropped by the renderer: rasterising the
                # whole sheet and cutting pieces out of it is how you turn a
                # 128 x 64 m floor into a 2 GB PIL image.
                from tools.track.svgfloor import (
                    render_window_via_converter as render_window)

                svg_path = pathlib.Path(track.warehouse.source).with_name(
                    "warehouse_map.svg")
                # `graph=None`: the lanes are geometry now, so this is the
                # warehouse *drawing* and nothing senses it. That is what
                # frees its resolution -- it no longer has to render a 20 mm
                # line wide enough for an IR array to find the centre of.
                image = render_window(svg_path, tile["origin_cm"],
                                      tile["size_m"], track.pixels_per_metre,
                                      graph=None)
            else:
                image = render_graph(graph, spec, track.line_width)

            source = pathlib.Path("textures/track.png")
            image.save(source)
            digest = hashlib.sha1(source.read_bytes()).hexdigest()[:12]
            # Content-addressed because the streaming viewer caches by URL: a
            # regenerated floor otherwise keeps arriving as the previous one.
            name = "track-{}.png".format(digest)
            (pathlib.Path("textures") / name).write_bytes(source.read_bytes())
            TRACK_TEXTURE.append(name)

        pathlib.Path("config").mkdir(exist_ok=True)
        document = to_document(graph,
                               node_spacing_cm=track.node_spacing_cm,
                               plane_m=track.plane_size)
        pathlib.Path("config/warehouse.json").write_text(
            json.dumps(document, indent=2) + "\n")
        print("textures/track.png ({}x{} px), config/warehouse.json "
              "({} nodes, {} edges)".format(spec.width_px, spec.height_px,
                                            len(graph.nodes), len(graph.edges)))

    if not args.skip_markers:
        # Stale tiles from a previous, larger track are not harmless: the world
        # only references the current nodes, but 881 leftover 1024x1024 files
        # are 3.7 GB of texture sitting in the tree, and the next reader
        # measuring "what does the fleet cost" counts them.
        for stale in pathlib.Path("textures/markers").glob("node_*.png"):
            stale.unlink()

        from tools.make_markers import main as make_markers

        # The *effective* manifest, not the file: with `--window` the two
        # differ, and reading the file again generated 881 tiles for an
        # 83-node graph -- 798 of them for nodes the world does not contain.
        import tempfile

        with tempfile.NamedTemporaryFile("w", suffix=".yaml",
                                         delete=False) as handle:
            yaml.safe_dump(manifest, handle)
            effective = handle.name
        try:
            make_markers(["--manifest", effective])
        finally:
            pathlib.Path(effective).unlink()

    pathlib.Path("worlds").mkdir(exist_ok=True)
    pathlib.Path("worlds/track.wbt").write_text(world_source(manifest))

    compose = compose_source(manifest)
    pathlib.Path("compose.fleet.yml").write_text(
        "# Generated by tools/gen_fleet.py -- edit fleet.yaml, not this file.\n"
        + yaml.safe_dump(compose, sort_keys=True)
    )

    config_dir = pathlib.Path("config")
    config_dir.mkdir(exist_ok=True)
    optics = optics_config(manifest)
    for config in robot_configs(manifest):
        document = {
            "name": config["name"],
            "sensors": {k: (list(v) if isinstance(v, tuple) else v)
                        for k, v in config["sensors"].items()},
            "control": config.get("control") or {},
            "optics": deep_merge(optics, config.get("optics") or {}),
            "odometry": config.get("odometry") or dict(ODOMETRY),
            "lidar": config.get("lidar") or {"enabled": True},
        }
        (config_dir / "{}.yaml".format(config["name"])).write_text(
            "# Generated by tools/gen_fleet.py -- edit fleet.yaml, not this file.\n"
            + yaml.safe_dump(document, sort_keys=True)
        )

    print("generated worlds/track.wbt, compose.fleet.yml, and {} config file(s)".format(
        len(robot_configs(manifest))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
