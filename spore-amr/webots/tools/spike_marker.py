"""Spike: place the robot on a marker and report what its sensors see.

Driving to a marker takes ten seconds and couples the question to the
line-follower. This teleports instead, so a failure here is unambiguously about
the marker being visible to a sensor, not about getting there.

Run as the supervisor:

    docker compose run --rm --entrypoint /usr/local/webots/webots-controller \\
      bot_01 --protocol=tcp --ip-address=sim --port=1234 \\
      --robot-name=supervisor /project/tools/spike_marker.py
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import yaml  # noqa: E402
from controller import Supervisor  # noqa: E402

from tools.manifest import MarkerConfig, TrackConfig  # noqa: E402


def sample(supervisor, robot_node, x, y, heading, timestep, label):
    """Put the robot at a pose and report every ground-facing sensor."""
    translation = robot_node.getField("translation")
    rotation = robot_node.getField("rotation")
    translation.setSFVec3f([x, y, 0.02])
    rotation.setSFRotation([0, 0, 1, heading])
    robot_node.resetPhysics()

    # Two steps: one to apply the move, one to render sensors at the new pose.
    for _ in range(4):
        supervisor.step(timestep)

    print("\n--- {} at ({:.3f}, {:.3f}) ---".format(label, x, y), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=pathlib.Path, default=pathlib.Path("fleet.yaml"))
    parser.add_argument("--robot", default="bot_01")
    args = parser.parse_args(argv)

    manifest = yaml.safe_load(args.manifest.read_text())
    track = TrackConfig.from_dict(manifest["track"])
    markers = MarkerConfig.from_dict(manifest.get("markers"))
    centerline = track.build_centerline()

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    robot_node = supervisor.getFromDef(args.robot.upper())
    if robot_node is None:
        print("no DEF {} in the world".format(args.robot.upper()), flush=True)
        return 2

    # The supervisor cannot read another robot's devices, so this reports what
    # is in the scene at each pose rather than what the robot's own sensors
    # return. Position of the marker Solids is the thing under test.
    for node in markers.nodes:
        x, y, heading = node.world_pose(centerline)
        marker_node = supervisor.getFromDef("MARKER_{}".format(node.node_id))
        if marker_node is None:
            print("marker {}: NO SUCH DEF in the world".format(node.node_id), flush=True)
            continue

        position = marker_node.getPosition()
        print("marker {} {}: solid at ({:.3f}, {:.3f}, {:.4f})  expected ({:.3f}, {:.3f})".format(
            node.node_id, node.kind, position[0], position[1], position[2], x, y), flush=True)

        # Where the robot's colour sensor would sit when centred on this marker.
        sample(supervisor, robot_node, x, y, heading, timestep,
               "robot on marker {}".format(node.node_id))

    print("\nthe robot is parked on the last marker; read its own sensors with "
          "spike_optics.py in a second controller", flush=True)
    for _ in range(400):
        if supervisor.step(timestep) == -1:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
