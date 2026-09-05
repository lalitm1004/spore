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

from robot.navigator import Navigator, load_map  # noqa: E402
from robot.uplink import Uplink  # noqa: E402
from robot.policy import CompanionPolicy  # noqa: E402
from robot.protocol import LineReader, Message, encode  # noqa: E402


def answer_junction(navigator, network, event, bot_id=0):
    """Turn a marker arrival into a TURN command, or into nothing.

    Two things happen here. The robot reports where it is, and it works out its
    own next move towards wherever the network layer last told it to go.

    That split is the contract's. `NetworkToRobot` carries `target_node_id` and
    nothing else, so a command is a **destination, not a direction** -- and a
    standing one: it holds until the robot arrives, which is what lets a robot
    be sent across the warehouse rather than steered corner by corner. The
    route to it is the robot's own work, because it holds the map too and can
    compute the next lane exactly.

    Re-planned from scratch at every marker rather than followed from a cached
    route: a robot pushed off its lane by the obstacle reflex is somewhere it
    did not expect to be, and re-planning from where it *is* costs one
    breadth-first sweep and is always right.

    Nothing is a legitimate answer. A robot nobody has given a goal sits still
    rather than inventing one -- that would be exactly the local autonomy the
    architecture says the network layer owns -- and the firmware's own junction
    timeout is what stops it waiting for ever.
    """
    node_id = int(event.fields.get("node", -1))
    if node_id < 0:
        return []

    if node_id not in navigator.graph.nodes:
        print("node {} is not in the map".format(node_id), flush=True)
        return []

    # Before `arrived` overwrites it: the heading the robot came in on, which
    # is the lane bearing from the previous node and owes nothing to odometry.
    arrived_on = navigator.heading_into(node_id)
    navigator.arrived(node_id)

    # Simulation milliseconds, not wall time. The firmware's clock is the only
    # one the run happens on, so a journal replays against the telemetry CSV
    # and `MISSION_DURATION` rather than against whatever the host was doing.
    timestamp = int(float(event.fields.get("t", 0.0)) * 1000)

    goal = network.report(bot_id, navigator.graph.nodes[node_id].region_id,
                          node_id, timestamp) if network is not None else None
    if goal is None:
        print("node {}: no destination from the network layer".format(node_id),
              flush=True)
        return []

    if goal == node_id:
        # Arrived. The network layer reconciles that from the next status and
        # sends somewhere new; there is nothing to drive in the meantime.
        print("node {}: destination reached".format(node_id), flush=True)
        return []

    hop = navigator.graph.next_hop(node_id, goal)
    if hop is None:
        navigator.bad_answers += 1
        print("node {}: no route to node {}".format(node_id, goal), flush=True)
        return []

    bearing = navigator.graph.bearing(node_id, hop)
    fields = {"bearing": round(bearing, 5), "node": hop, "goal": goal}
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
    parser.add_argument("--network", type=str, default=None,
                        help="host:port of the fleet's network layer")
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
    if navigator is None:
        print("no map: junctions will not be answered", flush=True)

    # The network layer is one service for the whole fleet, reached over gRPC.
    # `grpc` is imported inside NetworkClient, so a run without a network layer
    # costs nothing but the flag.
    network = None
    if args.network:
        from temp_network_interface.client import NetworkClient

        client = NetworkClient(args.network)
        client.connect()
        network = Uplink(client, wait_s=float(control.get("junction_timeout_s", 6.0)))
        print("network layer at {}".format(args.network), flush=True)
    else:
        print("no network layer: robots will sit at their first marker",
              flush=True)

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
