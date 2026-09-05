"""Ground truth and the on-screen readout.

Runs as its own extern controller with supervisor powers. Two jobs:

  1. Overlay each robot's last decoded marker as text in the 3D view, so a
     viewer can see what the fleet knows without reading a log.
  2. Report true pose against the robot's own belief, which is the only honest
     way to score localisation -- and the reason no robot carries a GPS. A
     privileged sensor on the robot is one the control code can quietly start
     depending on; keeping it here makes that impossible.

Robots publish their reads as small JSON files in the shared project volume.
That needs no new protocol, crosses container boundaries, and a lost update
costs nothing because another arrives at the next marker.
"""

import argparse
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from controller import Supervisor  # noqa: E402



# Label slots. Webots addresses overlay labels by index, so they are allocated
# statically rather than per-frame.
TITLE_LABEL = 0
FIRST_ROBOT_LABEL = 1

WHITE = 0xFFFFFF
AMBER = 0xFFA500
GREEN = 0x33DD55
KIND_COLORS = {
    "PT": 0x33DD55,
    "TR": 0x2A80FF,
    "CH": 0x1AE5E5,
    "PK": 0xB44DFF,
    "YI": 0xFF2A1A,
}
KIND_NAMES = {
    "PT": "pass-through",
    "TR": "transfer",
    "CH": "charging",
    "PK": "parking",
    "YI": "yield",
}


def read_status(path):
    """Latest marker read for one robot, or None if it has not read one yet."""
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=pathlib.Path, default=ROOT / "out")
    parser.add_argument("--map", type=pathlib.Path,
                        default=ROOT / "config" / "warehouse.json")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--calibrate", action="store_true",
                        help="log the raw heading terms for recalibration")
    args = parser.parse_args(argv)

    # The generated map, not the manifest's track. Building the track would
    # re-read the source warehouse.json, which lives outside the container --
    # and needing the source at runtime would defeat the point of generating a
    # map in the first place.
    document = json.loads(args.map.read_text())
    origin_offset = (document["dimensions"]["width"] / 200.0,
                     document["dimensions"]["height"] / 200.0)
    marker_count = len(document["nodes"])

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # gen_fleet.py writes each robot as `DEF <NAME_UPPER> LineBot`, which is
    # the only handle a supervisor gets on a PROTO instance.
    # The generated per-robot configs, not the manifest's `robots:` block --
    # that block may be a count-and-spawn rule rather than a list, and the
    # configs are what actually exist.
    names = sorted(path.stem for path in
                   (ROOT / "config").glob("bot_*.yaml"))
    nodes = {name: supervisor.getFromDef(name.upper()) for name in names}
    for name, node in nodes.items():
        if node is None:
            print("supervisor: no DEF {} in the world; "
                  "regenerate with tools.gen_fleet".format(name.upper()), flush=True)

    supervisor.setLabel(TITLE_LABEL,
                        "Floorgraph  |  {} markers  |  {} robots".format(
                            marker_count, len(names)),
                        0.01, 0.01, 0.07, WHITE, 0.0, "Arial")

    # The labels only exist in the 3D view, which a headless run has no way to
    # inspect. Echo each change to stdout so the readout is checkable from
    # `docker compose logs` as well as from the browser.
    last_text = {}
    last_read_id = {}
    fix_error = {}
    heading_error = {}
    wheel_drift = {}

    started = None
    while supervisor.step(timestep) != -1:
        now = supervisor.getTime()
        if started is None:
            started = now
        if args.duration is not None and now - started >= args.duration:
            break

        for index, name in enumerate(names):
            status = read_status(args.out / "{}.status.json".format(name))

            if status is None:
                text = "{}  searching...".format(name)
                colour = AMBER
            else:
                kind = status.get("kind", "??")
                text = "{}  node {} {} {}  ({}, {}) cm  region {}".format(
                    name,
                    status.get("node_id"),
                    KIND_NAMES.get(kind, kind),
                    status.get("name", ""),
                    status.get("x_cm"),
                    status.get("y_cm"),
                    status.get("region_id"),
                )
                colour = KIND_COLORS.get(kind, WHITE)

            # Localisation error is only meaningful at the instant of a read.
            # Sampling it every step measures how far the robot has driven
            # since the marker, which is not an error at all.
            node = nodes.get(name)
            if status is not None and node is not None:
                read_id = (status.get("node_id"), status.get("t"))
                if read_id != last_read_id.get(name):
                    last_read_id[name] = read_id
                    position = node.getPosition()
                    true_x = (position[0] + origin_offset[0]) * 1000
                    true_y = (position[1] + origin_offset[1]) * 1000
                    believed_x = status.get("fix_x_mm")
                    believed_y = status.get("fix_y_mm")
                    if believed_x is None:
                        # No lever-arm fix, so fall back to the marker's own
                        # position -- cm in the shared schema, mm here.
                        believed_x = status["x_cm"] * 10.0
                        believed_y = status["y_cm"] * 10.0
                    fix_error[name] = math.hypot(true_x - believed_x,
                                                  true_y - believed_y)

                    # Attribute the error: a lever-arm fix rotates by the
                    # robot's believed heading, so heading drift shows up
                    # here multiplied by the boom length.
                    believed_theta = status.get("odo_theta")
                    if believed_theta is not None:
                        m = node.getOrientation()
                        true_theta = math.atan2(m[3], m[0])
                        drift = (believed_theta - true_theta + math.pi) % (2 * math.pi) - math.pi
                        heading_error[name] = math.degrees(drift)
                        drifted = status.get("drifted_theta")
                        if drifted is not None:
                            wheels = (drifted - true_theta + math.pi) % (2 * math.pi) - math.pi
                            wheel_drift[name] = math.degrees(wheels)

                        if args.calibrate:
                            # The raw ingredients of a marker-derived heading,
                            # for re-deriving Optics.CAMERA_ROTATION_ZERO after
                            # any change to how the camera is mounted.
                            print("heading-calib t={} true={:.4f} believed={:.4f} "
                                  "tile_bearing={} image_rot={:.4f}".format(
                                      status.get("t"), true_theta, believed_theta,
                                      status.get("bearing_deg"),
                                      status.get("image_rotation")), flush=True)
                if name in fix_error:
                    text += "   fix error {:.0f} mm".format(fix_error[name])
                if name in wheel_drift:
                    text += "  wheels {:+.1f} deg".format(wheel_drift[name])
                if name in heading_error:
                    text += "  heading {:+.1f} deg".format(heading_error[name])

            supervisor.setLabel(FIRST_ROBOT_LABEL + index, text,
                                0.01, 0.06 + 0.045 * index, 0.06, colour, 0.0, "Arial")

            if last_text.get(name) != text:
                last_text[name] = text
                print("label: {}".format(text), flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
