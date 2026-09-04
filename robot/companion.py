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

import yaml  # noqa: E402

from robot.policy import CompanionPolicy  # noqa: E402
from robot.protocol import LineReader, encode  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--link", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--mission-duration", type=float, default=120.0)
    args = parser.parse_args(argv)

    document = yaml.safe_load(args.config.read_text())
    control = document.get("control") or {}
    cruise = float(control.get("base_speed", 6.0))

    policy = CompanionPolicy(
        cruise_speed=cruise,
        min_speed=max(1.0, cruise * 0.25),
        slowdown=0.6,
        mission_duration_s=args.mission_duration,
    )
    reader = LineReader()

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
                for command in policy.on_event(event):
                    link.write(encode(command))
                    print("-> {} {}".format(command.name, command.fields), flush=True)
                    if command.name == "STOP":
                        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
