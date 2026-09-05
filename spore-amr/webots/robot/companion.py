"""Companion: the Pi Zero's job.

Opens the serial link exactly as it would open /dev/ttyACM0, reacts to the
events the firmware reports, and issues commands. It has no access to the
simulator, no sensors and no motors -- the same as on the real robot.
"""

import argparse
import pathlib
import select
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import yaml  # noqa: E402

from robot.navigator import NetworkLink, Navigator, load_map  # noqa: E402
from robot.policy import CompanionPolicy  # noqa: E402
from robot.protocol import LineReader, Message, encode  # noqa: E402


def answer_junction(navigator, network, event, bot_id=0):
    """Turn a marker arrival into a TURN command, or into nothing.

    Nothing is a legitimate answer. A robot nobody answers should sit still
    rather than pick a direction for itself -- inventing one here would be
    exactly the local autonomy the architecture says the network layer owns.
    The firmware's own junction timeout is what stops it waiting forever.

    A dead end is no longer one of those cases. The query names the node the
    robot is at and the answer names the node to go to, so a charging bay's one
    neighbour is simply the only answer available; there is no menu of turns
    left for the way back out to fall off the end of.
    """
    node_id = int(event.fields.get("node", -1))
    if node_id < 0:
        return []

    try:
        query = navigator.build_query(
            node_id, bot_id=bot_id,
            timestamp=int(time.time() * 1000))
    except KeyError:
        print("node {} is not in the map".format(node_id), flush=True)
        return []

    # Before `arrived` overwrites it: the heading the robot came in on, which
    # is the lane bearing from the previous node and owes nothing to odometry.
    arrived_on = navigator.heading_into(node_id)
    navigator.arrived(node_id)

    decision = network.ask(query) if network is not None else None
    if decision is None:
        print("node {}: no answer from the network layer".format(node_id),
              flush=True)
        return []

    bearing = navigator.bearing_for(node_id, decision)
    if bearing is None:
        # The network layer named somewhere this node has no lane to. Not a
        # dead end -- a wrong answer, and the robot will not drive it.
        navigator.bad_answers += 1
        print("node {}: node {} is not a lane from here".format(
            node_id, decision.target_node_id), flush=True)
        return []

    fields = {
        "bearing": round(bearing, 5),
        "node": decision.target_node_id,
    }
    # The firmware turns to an absolute bearing, so without this it turns in a
    # drifted frame and lands on a lane nobody chose.
    if arrived_on is not None:
        fields["heading"] = round(arrived_on, 5)

    return [Message(kind="CMD", name="TURN", fields=fields)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--link", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--mission-duration", type=float, default=120.0)
    parser.add_argument("--map", type=pathlib.Path, default=None,
                        help="warehouse.json; without it the robot cannot turn")
    parser.add_argument("--netlayer", type=pathlib.Path, default=None,
                        help="unix socket of this robot's network layer")
    args = parser.parse_args(argv)

    document = yaml.safe_load(args.config.read_text())
    # `bot_id` is an integer on the wire, and the fleet's names are bot_NN.
    digits = "".join(c for c in str(document.get("name", "")) if c.isdigit())
    bot_id = int(digits) if digits else 0
    control = document.get("control") or {}
    cruise = float(control.get("base_speed", 6.0))

    policy = CompanionPolicy(
        cruise_speed=cruise,
        min_speed=max(1.0, cruise * 0.25),
        slowdown=0.6,
        mission_duration_s=args.mission_duration,
        # Symmetric with `slowdown`: what a loss takes off, a clean run of
        # line puts back. Without this the throttle only ever went one way.
        recover_after_s=float(control.get("speed_recover_after_s", 5.0)),
        speedup=1.0 / 0.6,
    )
    reader = LineReader()

    # The map lives here, not in the firmware: the firmware drives and must
    # not acquire a dependency on a file or a socket.
    navigator = Navigator(load_map(args.map)) if args.map and args.map.exists() else None
    network = NetworkLink(args.netlayer) if args.netlayer else None
    if navigator is None:
        print("no map: junctions will not be answered", flush=True)

    with open(args.link, "r+b", buffering=0) as link:
        for command in policy.start():
            link.write(encode(command))
            print("-> {} {}".format(command.name, command.fields), flush=True)

        while True:
            ready, _, _ = select.select([link], [], [], 5.0)
            if not ready:
                continue

            chunk = link.read(4096)
            if not chunk:
                break

            for event in reader.feed(chunk):
                if event.name != "STATUS":
                    print("<- {} {}".format(event.name, event.fields), flush=True)

                if event.name == "MARKER" and navigator is not None:
                    for command in answer_junction(navigator, network, event,
                                                  bot_id=bot_id):
                        link.write(encode(command))
                        print("-> {} {}".format(command.name, command.fields),
                              flush=True)

                for command in policy.on_event(event):
                    link.write(encode(command))
                    print("-> {} {}".format(command.name, command.fields), flush=True)
                    if command.name == "STOP":
                        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
