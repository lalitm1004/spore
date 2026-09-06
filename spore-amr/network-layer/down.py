"""Tear down fleet containers, and the networks left empty behind them.

    uv run down.py               # everything
    uv run down.py --region 2    # one region only

Also imported by `tests/test_docker.py`, which sweeps before it starts. Teardown
is not guaranteed -- interrupt a run and the containers outlive the process that
made them -- and a leftover fleet is not idle: every bot in it goes on
heartbeating on a timer. A dozen strays turn a one-minute suite into three while
looking like nothing is wrong.

Everything here filters on `amr.fleet`, which only `up.py` sets, so nothing that
is not ours is touched.
"""
from __future__ import annotations

import argparse

import docker
from docker.errors import NotFound

from up import LABEL_FLEET, LABEL_REGION


def remove_fleet(client: docker.DockerClient, region: int | None = None,
                 quiet: bool = False) -> int:
    """Remove every fleet container, and any of our networks left empty.

    Returns how many containers went. Networks are found by label rather than
    by name: a test run makes one per fleet with a random tag, so there is no
    single name to pass in, and leaving them behind leaks a subnet each time.
    """
    filters = {"label": [LABEL_FLEET]}
    if region is not None:
        filters["label"].append(f"{LABEL_REGION}={region}")

    containers = client.containers.list(all=True, filters=filters)
    for c in containers:
        if not quiet:
            print(f"removing {c.name}")
        try:
            c.remove(force=True)  # SIGKILL — bots get no chance to send Departure
        except NotFound:
            pass                  # someone else got there first

    for net in client.networks.list(filters={"label": [LABEL_FLEET]}):
        try:
            net.reload()
            if not net.containers:
                if not quiet:
                    print(f"removing network {net.name}")
                net.remove()
        except NotFound:
            pass

    return len(containers)


def main() -> None:
    p = argparse.ArgumentParser(description="Stop and remove AMR bot containers")
    p.add_argument("--region", type=int, default=None)
    args = p.parse_args()

    removed = remove_fleet(docker.from_env(), region=args.region)
    if not removed:
        print("no fleet containers found")


if __name__ == "__main__":
    main()
