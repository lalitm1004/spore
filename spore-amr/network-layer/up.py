"""Local orchestrator: build the bot image and launch N bots on one flat network.

WHAT
    `uv run up.py --bots 3 --region 14` builds `amr-bot:dev`, makes sure the
    bridge network exists, and starts N containers with the identity env vars
    from PROTOCOL.md §2. Later invocations add bots (any region) that can see
    the ones already running. `down.py` tears it all down.

WHERE
    Also imported by `tests/test_docker.py`, which uses `launch()` with its
    own network and name prefix so test fleets never collide with a dev fleet.

WHY
    All bots share a single bridge network (like one WiFi IRL); region
    isolation is enforced in-process by `bus/policy.py`, not by Docker
    (PROTOCOL.md §12, §13). Docker's embedded DNS resolves container names,
    so `OWN_ADDRESS` is just `<container-name>:50051`.

HOW
    * Every container carries labels `amr.fleet=1`, `amr.region=<id>` and
      `amr.net=<network>`; `down.py` and the test harness find fleet
      containers by label and never touch anything else.
    * `PEER_LEADERS` for a new batch = every fleet container already running
      on that network + its batch-mates, so a second region discovers the
      first (PROTOCOL.md §4.1).
    * `BOT_ID` continues from the highest running id unless `--start-id` is
      given — ids must be fleet-unique (they are the election tiebreak).
    * The warehouse map lives outside this build context, so it is
      bind-mounted read-only rather than baked into the image.
    * `ADMIN_ENABLED=1` so `AdminService` (introspection, robot-state
      injection) is available locally.
"""
from __future__ import annotations

import argparse
import os
import sys

import docker
from docker.errors import APIError, ImageNotFound, NotFound

IMAGE = "amr-bot:dev"
GRPC_PORT = 50051
LABEL_FLEET = "amr.fleet"
LABEL_REGION = "amr.region"
LABEL_NET = "amr.net"

# network-layer/ -> spore-amr/ -> shared/. The extra "spore-amr" this used to
# carry was right when network-layer sat at the repo root; the move left it a
# level too deep, and a missing file here silently skips the mount, so every
# container ran geography-blind.
MAP_HOST_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "shared", "warehouse-layout.json"))
MAP_CONTAINER_PATH = "/app/warehouse-layout.json"


def container_name(region_id: int, bot_id: int, prefix: str = "amr") -> str:
    return f"{prefix}-region-{region_id}-bot-{bot_id}"


def build_image(client: docker.DockerClient, quiet: bool = False) -> None:
    if not quiet:
        print(f"building {IMAGE} ...")
    _, logs = client.images.build(path=os.path.dirname(os.path.abspath(__file__)), tag=IMAGE, rm=True)
    for chunk in logs:
        line = chunk.get("stream", "").rstrip()
        if line and not quiet:
            print(f"  {line}")


def ensure_network(client: docker.DockerClient, name: str):
    try:
        return client.networks.get(name)
    except NotFound:
        print(f"creating network {name}")
        return client.networks.create(name, driver="bridge", labels={LABEL_FLEET: "1"})


def fleet_containers(client: docker.DockerClient, network: str, all_states: bool = False):
    return client.containers.list(all=all_states, filters={"label": [LABEL_FLEET, f"{LABEL_NET}={network}"]})


def existing_fleet_addresses(client: docker.DockerClient, network: str) -> list[str]:
    """Bots already running on this network — new bots need them for bootstrap discovery."""
    return [f"{c.name}:{GRPC_PORT}" for c in fleet_containers(client, network)]


def next_bot_id(client: docker.DockerClient, network: str) -> int:
    """One past the highest BOT_ID running on this network."""
    highest = -1
    for c in fleet_containers(client, network):
        env = dict(e.split("=", 1) for e in c.attrs["Config"]["Env"] if "=" in e)
        highest = max(highest, int(env.get("BOT_ID", -1)))
    return highest + 1


def remove_if_ours(client: docker.DockerClient, name: str) -> None:
    try:
        c = client.containers.get(name)
    except NotFound:
        return
    if c.labels.get(LABEL_FLEET) != "1":
        sys.exit(f"refusing to replace {name}: it is not a fleet container")
    print(f"  replacing existing {name}")
    c.remove(force=True)


def host_endpoint(container) -> str:
    """`host:port` the *host* can dial to reach this container.

    Containers talk to each other by name on the bridge network; this is only for
    code running outside Docker. Falls back to the container IP, which works on
    Linux, when no port was published.
    """
    container.reload()
    bindings = (container.attrs["NetworkSettings"]["Ports"] or {}).get(f"{GRPC_PORT}/tcp")
    if bindings:
        return f"127.0.0.1:{bindings[0]['HostPort']}"
    networks = container.attrs["NetworkSettings"]["Networks"]
    ip = next(iter(networks.values()))["IPAddress"]
    return f"{ip}:{GRPC_PORT}"


def launch(
    client: docker.DockerClient, num_bots: int, region_id: int, network: str,
    start_id: int | None = None, prefix: str = "amr", extra_env: dict[str, str] | None = None,
    quiet: bool = False,
) -> list:
    """Start `num_bots` containers in `region_id`. Returns the Container objects."""
    if start_id is None:
        start_id = next_bot_id(client, network)
    already_running = existing_fleet_addresses(client, network)
    new_names = [container_name(region_id, start_id + i, prefix) for i in range(num_bots)]
    all_addrs = already_running + [f"{n}:{GRPC_PORT}" for n in new_names]
    volumes = {MAP_HOST_PATH: {"bind": MAP_CONTAINER_PATH, "mode": "ro"}} if os.path.isfile(MAP_HOST_PATH) else {}

    started = []
    for i, name in enumerate(new_names):
        bot_id = start_id + i
        own = f"{name}:{GRPC_PORT}"
        peers = ",".join(a for a in all_addrs if a != own)
        remove_if_ours(client, name)
        env = {
            "BOT_ID": str(bot_id),
            "REGION_ID": str(region_id),
            "OWN_ADDRESS": own,
            "PEER_LEADERS": peers,
            "GRPC_PORT": str(GRPC_PORT),
            "GRPC_HOST": "0.0.0.0",
            "WAREHOUSE_MAP": MAP_CONTAINER_PATH,
            "ADMIN_ENABLED": "1",
            "PYTHONUNBUFFERED": "1",
        }
        env.update(extra_env or {})
        try:
            c = client.containers.run(
                IMAGE, name=name, network=network, detach=True, volumes=volumes, environment=env,
                # Publish the gRPC port on an ephemeral host port. Bots reach each
                # other by container name on the bridge and never use this; it is
                # for the operator and the test suite, which run on the host.
                # Docker Desktop on macOS does not route to container IPs at all,
                # so without this the whole Docker tier is unrunnable there.
                ports={f"{GRPC_PORT}/tcp": None},
                labels={LABEL_FLEET: "1", LABEL_REGION: str(region_id), LABEL_NET: network},
            )
            started.append(c)
            if not quiet:
                print(f"  started {name} ({c.short_id})")
        except APIError as e:
            print(f"  failed {name}: {e.explanation}", file=sys.stderr)
    return started


def main() -> None:
    p = argparse.ArgumentParser(description="Launch AMR bots on a flat Docker network")
    p.add_argument("--bots", type=int, default=3)
    p.add_argument("--region", type=int, default=14)
    p.add_argument("--start-id", type=int, default=None, help="first BOT_ID (default: continue after highest running)")
    p.add_argument("--network", default="amr-net")
    p.add_argument("--no-build", action="store_true", help="reuse the existing image")
    args = p.parse_args()

    client = docker.from_env()
    if args.no_build:
        try:
            client.images.get(IMAGE)
        except ImageNotFound:
            sys.exit(f"{IMAGE} not found; run without --no-build first")
    else:
        build_image(client)

    ensure_network(client, args.network)
    started = launch(client, args.bots, args.region, args.network, args.start_id)
    print(f"\n{len(started)} bot(s) in region {args.region} on {args.network}")
    if started:
        print(f"  docker logs -f {started[0].name}")


if __name__ == "__main__":
    main()
