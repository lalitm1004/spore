"""The network layer, as its own process.

One per robot, not one shared service: the architecture's claim is that
coordination is distributed, and a single service answering every robot would
be a control plane wearing a hat. Running one per robot also makes killing a
single robot's coordinator a real thing to demonstrate.

It listens on a unix socket and answers newline-delimited JSON. That is a real
serialisation boundary -- the messages on it are the shared schemas, encoded
and decoded -- so replacing this with the real TypeScript network layer means
changing what listens on the socket and nothing else.

The companion talks to this; the firmware never does. The firmware's job is to
drive, and it must not acquire a network dependency.

It loads the warehouse map, because choosing where a robot goes next is what a
network layer is for. The robot sends `latest_node_id` and gets back
`target_node_id`; it never offers a menu of turns, so there is nothing to
choose from except the map. Working out that the target is a left turn is the
robot's own job -- it holds the map too, and can do it exactly.
"""

import argparse
import os
import pathlib
import socket
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robot.navigator import load_map  # noqa: E402
from robot.network import Query, RandomRouter  # noqa: E402


def serve(path: pathlib.Path, router, verbose: bool = True) -> int:
    if path.exists():
        path.unlink()
    path.parent.mkdir(parents=True, exist_ok=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(1)
    os.chmod(str(path), 0o777)
    print("netlayer[{}] listening on {}".format(router.name, path), flush=True)

    try:
        while True:
            connection, _ = server.accept()
            with connection:
                handle(connection, router, verbose)
    except KeyboardInterrupt:
        return 0
    finally:
        server.close()
        if path.exists():
            path.unlink()


def handle(connection, router, verbose: bool) -> None:
    """Answer queries until the peer goes away.

    A companion that dies simply closes the socket; that is not an error here,
    and the next one connects to the same listener.
    """
    buffer = b""
    while True:
        try:
            chunk = connection.recv(4096)
        except OSError:
            return
        if not chunk:
            return

        buffer += chunk
        while b"\n" in buffer:
            line, _, buffer = buffer.partition(b"\n")
            text = line.decode("utf-8", "replace").strip()
            if not text:
                continue

            try:
                query = Query.from_json(text)
            except (ValueError, KeyError) as error:
                print("netlayer: bad query ({}): {}".format(error, text[:80]),
                      flush=True)
                continue

            decision = router.route(query)
            if decision is None:
                # Nowhere legal to send it. Say nothing rather than invent a
                # target: the robot's own timeout is the right thing to fire.
                if verbose:
                    print("netlayer: node {} is not on the map, no answer"
                          .format(query.latest_node_id), flush=True)
                continue

            if verbose:
                print("netlayer: bot {} at node {} -> node {}".format(
                    query.bot_id, query.latest_node_id,
                    decision.target_node_id), flush=True)
            try:
                connection.sendall((decision.to_json() + "\n").encode("utf-8"))
            except OSError:
                return


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=pathlib.Path, required=True)
    parser.add_argument("--map", type=pathlib.Path,
                        default=ROOT / "config" / "warehouse.json",
                        help="the network layer routes, so it holds the map")
    parser.add_argument("--seed", type=int, default=0,
                        help="a run that cannot be reproduced cannot be debugged")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    return serve(args.socket, RandomRouter(load_map(args.map), seed=args.seed),
                 verbose=not args.quiet)


if __name__ == "__main__":
    raise SystemExit(main())
