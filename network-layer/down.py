"""Tear down fleet containers (and the network, if nothing else is on it).

    uv run down.py               # everything
    uv run down.py --region 2    # one region only
"""
from __future__ import annotations

import argparse

import docker
from docker.errors import NotFound

from up import LABEL_FLEET, LABEL_REGION


def main() -> None:
    p = argparse.ArgumentParser(description="Stop and remove AMR bot containers")
    p.add_argument("--region", type=int, default=None)
    p.add_argument("--network", default="amr-net")
    args = p.parse_args()

    client = docker.from_env()

    filters = {"label": [LABEL_FLEET]}
    if args.region is not None:
        filters["label"].append(f"{LABEL_REGION}={args.region}")

    containers = client.containers.list(all=True, filters=filters)
    if not containers:
        print("no fleet containers found")
    for c in containers:
        print(f"removing {c.name}")
        c.remove(force=True)  # SIGKILL — bots get no chance to send Departure

    try:
        net = client.networks.get(args.network)
        net.reload()
        if not net.containers:
            print(f"removing network {args.network}")
            net.remove()
    except NotFound:
        pass


if __name__ == "__main__":
    main()
