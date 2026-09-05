"""Command-line entry points.

    python -m temp_network_interface serve --address [::]:50051
    python -m temp_network_interface probe --target localhost:50051 --bot-id 1
"""

from __future__ import annotations

import argparse
import time

from temp_network_interface.messages import Mission, RobotToNetwork, Telemetry, Battery
from temp_network_interface.policy import HoldPolicy, NoopPolicy

# `goal` needs a map -- it routes -- so it is built separately below rather
# than sitting in this table of no-argument policies.
_POLICIES = {"hold": HoldPolicy, "noop": NoopPolicy}
_MAP_POLICIES = ("goal",)


def _build_policy(args):
    if args.policy not in _MAP_POLICIES:
        return _POLICIES[args.policy]()

    if not args.map:
        raise SystemExit("--policy {} needs --map: it routes, and routing "
                         "needs the warehouse graph".format(args.policy))

    import random

    from temp_network_interface.graph import load_map
    from temp_network_interface.goal_policy import GoalPolicy

    graph = load_map(args.map)
    return GoalPolicy(graph, minimum_hops=args.minimum_hops,
                      random=random.Random(args.seed))


def _serve(args) -> int:
    from temp_network_interface.server import serve

    serve(address=args.address, journal=args.journal, policy=_build_policy(args))
    return 0


def _probe(args) -> int:
    from temp_network_interface.client import NetworkClient

    with NetworkClient(args.target) as client:
        status = RobotToNetwork(
            bot_id=args.bot_id,
            region_id=args.region_id,
            latest_node_id=args.node_id,
            mission=Mission(type="IDLE"),
            telemetry=Telemetry(battery=Battery(percentage=100.0)),
            timestamp=int(time.time()),
        )
        client.send(status)
        command = client.recv(timeout=5.0)
        print("command: {}".format(command))

    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="temp-network-interface", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    serve = sub.add_parser("serve", help="run the network service")
    serve.add_argument("--address", default="[::]:50051")
    serve.add_argument("--policy",
                       choices=sorted(set(_POLICIES) | set(_MAP_POLICIES)),
                       default="hold")
    serve.add_argument("--map", default=None,
                       help="warehouse.json; required by routing policies")
    serve.add_argument("--minimum-hops", type=int, default=40,
                       help="how far away a destination has to be, in lanes")
    serve.add_argument("--seed", type=int, default=0,
                       help="a run that cannot be reproduced cannot be debugged")
    serve.add_argument("--journal", default=None,
                       help="path to the durable state file (JSONL)")
    serve.set_defaults(func=_serve)

    probe = sub.add_parser("probe", help="send one status and print the reply")
    probe.add_argument("--target", default="localhost:50051")
    probe.add_argument("--bot-id", type=int, default=1)
    probe.add_argument("--region-id", type=int, default=1)
    probe.add_argument("--node-id", type=int, default=10)
    probe.set_defaults(func=_probe)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
