"""Companion: the Pi Zero's job.

Opens the serial link exactly as it would open /dev/ttyACM0, reacts to the
events the firmware reports, and issues commands. It has no access to the
simulator, no sensors and no motors -- the same as on the real robot.
"""

import argparse
import pathlib
import select
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from robot.config import ControlConfig  # noqa: E402
from robot.navigator import Navigator, load_map  # noqa: E402
from robot.uplink import Uplink  # noqa: E402
from robot.policy import CompanionPolicy  # noqa: E402
from robot.protocol import LineReader, Message, encode  # noqa: E402


def report_obstacle(navigator, network, event):
    """Tell the network layer about something in the lane, or that it has gone.

    The robot's own reflex has already dealt with it -- stop, reverse to the
    last marker, hold -- so this is not a cry for help. It is the one thing only
    this robot knows and every robot planning through here needs: that the lane
    out of this node is not usable.

    The node is the last marker read, which is exactly where the reflex reverses
    to, so it is where the robot actually is when it reports. Sending it is what
    makes an obstruction real: the shared schema has always carried
    `current_node_id` on an OBSTACLE warning and nothing ever filled it in, so
    the planner only ever heard about blockages through an admin back door.

    `EVT OBSTACLE` fires on every change of state, clearing included. A report
    with no obstacle in it is how the lane is given back.
    """
    if network is None or navigator is None or navigator.last_node is None:
        return []

    blocked = event.fields.get("state", "CLEAR") != "CLEAR"
    node = navigator.last_node
    region = navigator.graph.nodes[node].region_id if node in navigator.graph.nodes else 0
    # Zero is how the lane is given back: present, and not a node.
    network.report(node_id=node, region_id=region,
                   obstacle_node=node if blocked else 0)
    return []


#: How far through a job each cargo state is. The robot's own state never goes
#: backwards along this, so a stale echo from the network cannot undo work the
#: robot has already done.
_CARGO_ORDER = {"PICKUP": 0, "EN_ROUTE": 1, "DROPOFF": 2}


def answer_junction(navigator, network, event):
    """Turn a marker arrival into a TURN command, or into nothing.

    Nothing is a legitimate answer. A robot with no network layer, or one at a
    dead end nobody will route out of, should sit still rather than pick a
    direction for itself -- inventing one here would be exactly the local
    autonomy the architecture says the network layer owns.
    """
    node_id = int(event.fields.get("node", -1))
    heading = float(event.fields.get("heading", 0.0))
    if node_id < 0:
        return []

    try:
        query = navigator.build_query(node_id, heading)
    except KeyError:
        print("node {} is not in the map".format(node_id), flush=True)
        return []

    # Before `arrived` overwrites it: the heading the robot came in on, which is
    # the lane bearing from the previous node and owes nothing to odometry.
    arrived_on = navigator.heading_into(node_id)
    navigator.arrived(node_id)

    if not query.available:
        navigator.dead_ends += 1
        print("node {}: nowhere to go from here".format(node_id), flush=True)
        return []

    decision = network.ask(query) if network is not None else None
    if decision is None:
        print("node {}: no answer from the network layer".format(node_id),
              flush=True)
        return []

    # Handling the cargo. This is the robot's half of the job cycle and without
    # it a job stops at its collection node: the network layer only moves the
    # goal to the delivery node once the robot reports CARGO/EN_ROUTE, and only
    # marks a job delivered once the robot stops reporting CARGO at all.
    #
    # A mission on the answer replaces what we were carrying; an answer with no
    # mission -- every WAIT, every plain PROCEED -- says nothing about cargo and
    # must leave it alone.
    # A network stand-in that does not model cargo simply does not carry any:
    # the tests use one, and so would any transport that never sends a mission.
    carries_cargo = network is not None and hasattr(network, "cargo_state")

    if carries_cargo and decision.mission:
        # Ignore a mission for cargo already delivered. The bot keeps sending
        # `set_mission` from its own job until it has read the delivery, and
        # several of its answers are already in flight when we put the cargo
        # down -- so without this the robot is handed the finished job back and
        # delivers it again, once per answer. Measured: 5 collections, 58
        # deliveries.
        if decision.cargo_id and decision.cargo_id == getattr(
                network, "delivered_cargo_id", ""):
            pass
        elif decision.cargo_id != network.cargo_id:
            # A different job: adopt it whole.
            network.mission = decision.mission
            network.cargo_id = decision.cargo_id
            network.cargo_state = decision.cargo_state
            network.collected_at = None
        else:
            # The same job. Cargo state only ever moves forward, because the
            # bot goes on echoing the state it last read from us and that read
            # is behind. Adopting it verbatim walked the robot backwards --
            # EN_ROUTE back to PICKUP -- and it collected the same cargo again
            # on the next answer, once per answer still in flight.
            network.mission = decision.mission
            if _CARGO_ORDER.get(decision.cargo_state, -1) > \
                    _CARGO_ORDER.get(network.cargo_state, -1):
                network.cargo_state = decision.cargo_state

    # Arriving is the network telling us we are there. A PROCEED naming this
    # node would say so, but the planner does not send one: `ALREADY_THERE`
    # becomes a WAIT with `because="at the goal"` and no target at all, so
    # keying on `target_node_id` alone matched nothing and every robot sat on
    # its pickup being told to hold. Both forms are accepted.
    arrived = decision.target_node_id == node_id or (
        decision.is_wait and "at the goal" in decision.because)

    if carries_cargo and network.mission == "CARGO" and arrived:
        # Standing on the node the job named. There is no manipulator to
        # simulate, so collecting and delivering are the reports themselves.
        if network.cargo_state == "PICKUP":
            network.cargo_state = "EN_ROUTE"
            # Where it was collected, because the robot does not leave the node
            # the instant it has the cargo. The bot's goal is still the pickup
            # until it reads this report, so the next few answers are still
            # "at the goal" -- and by then we are EN_ROUTE, which is the
            # delivery branch. Without this the cargo is put back down where it
            # was picked up, one node into a two-node journey.
            network.collected_at = node_id
            print("node {}: collected cargo {}".format(
                node_id, network.cargo_id or "?"), flush=True)
        elif network.cargo_state in ("EN_ROUTE", "DROPOFF") \
                and node_id != getattr(network, "collected_at", None):
            network.cargo_state = "DROPOFF"
            print("node {}: delivered cargo {}".format(
                node_id, network.cargo_id or "?"), flush=True)
            # Report the delivery once, then go idle: the network layer reads
            # "was DROPOFF, now not carrying" as the job being done.
            network.report(node_id, query.region_id)
            network.delivered_cargo_id = network.cargo_id
            network.collected_at = None
            network.mission = ""
            network.cargo_id = ""
            network.cargo_state = ""
        network.report(node_id, query.region_id)

    if decision.is_wait:
        # Hold, then ask the same question again. The firmware keeps the robot
        # where it is; nothing needs to be commanded to stand still.
        print("node {}: holding {} ms ({})".format(
            node_id, decision.hold_ms, decision.because or "no reason given"), flush=True)
        return [Message(kind="CMD", name="HOLD", fields={"ms": decision.hold_ms})]

    bearing = navigator.bearing_for(node_id, decision)
    if bearing is None:
        print("node {}: node {} is not a lane from here".format(
            node_id, decision.target_node_id), flush=True)
        return []

    fields = {
        "bearing": round(bearing, 5),
        "node": decision.target_node_id,
    }
    # The firmware turns to an *absolute* bearing and its only feedback is the
    # odometry heading, so the two have to share a frame. Without this it turns
    # in a drifted one and lands on a lane nobody chose -- measured against the
    # supervisor, all ten robots were out by exactly their spawn bearing.
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
    parser.add_argument("--network", default=None,
                        help="host:port of this robot's own network-layer bot")
    args = parser.parse_args(argv)

    # Imported here rather than at the top: `answer_junction` and
    # `report_obstacle` are pure, and being able to import them without pulling
    # in a YAML parser is what lets the integration test drive them from the
    # network layer's side of the fence.
    import yaml

    document = yaml.safe_load(args.config.read_text())
    control = document.get("control") or {}
    defaults = ControlConfig()
    cruise = float(control.get("base_speed", defaults.base_speed))
    laden = control.get("laden_speed", defaults.laden_speed)
    laden = cruise if laden is None else float(laden)
    # The uplink gives up at the same moment the firmware does. Any longer and
    # the companion is still waiting on a question the robot has already
    # answered by driving off; any shorter and it abandons replies that would
    # have arrived in time.
    patience = float(control.get("junction_timeout_s", defaults.junction_timeout_s))

    policy = CompanionPolicy(
        cruise_speed=cruise,
        laden_speed=laden,
        min_speed=max(1.0, cruise * 0.25),
        slowdown=0.6,
        mission_duration_s=args.mission_duration,
        # Configured since the ratchet was found and never passed in, so the
        # way back up was dead code: a robot that lost the line once stayed
        # slow for the rest of the run. `speed_recover_after_s` is in every
        # generated config already.
        recover_after_s=float(control.get(
            "speed_recover_after_s", defaults.speed_recover_after_s)),
        speedup=1.5,
    )
    reader = LineReader()

    # The map lives here, not in the firmware: the firmware drives and must
    # not acquire a dependency on a file or a socket.
    navigator = Navigator(load_map(args.map)) if args.map and args.map.exists() else None
    network = Uplink(args.network, timeout_s=patience) if args.network else None
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

                if event.name == "OBSTACLE":
                    report_obstacle(navigator, network, event)

                if event.name == "MARKER" and navigator is not None:
                    for command in answer_junction(navigator, network, event):
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
