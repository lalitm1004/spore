"""Command-line entry points.

    python -m temp_network_interface serve --address [::]:50051
    python -m temp_network_interface probe --target localhost:50051 --bot-id 1
"""

from __future__ import annotations

import argparse
import time

from temp_network_interface.messages import Mission, RobotToNetwork, Telemetry, Battery
from temp_network_interface.policy import HoldPolicy, NoopPolicy

_POLICIES = {"hold": HoldPolicy, "noop": NoopPolicy}


def _serve(args) -> int:
    from temp_network_interface.server import serve

    policy = _POLICIES[args.policy]()
    serve(address=args.address, journal=args.journal, policy=policy)
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
    serve.add_argument("--policy", choices=sorted(_POLICIES), default="hold")
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
