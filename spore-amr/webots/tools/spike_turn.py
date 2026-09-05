"""Spike: how accurate is an in-place turn?

`track_width` was calibrated from driving curves, which mix translation with
rotation. A spin in place is a different regime -- maximum lateral slip at the
tyres, no forward motion to stabilise it -- so the number that fixed heading
drift on the oval may not be the number that makes a 90 degree turn land.

Every junction depends on this, so it is worth measuring before there are any
junctions to depend on it.

Runs as bot_01, alongside tools/spike_turn_truth.py on the supervisor. Each
completed turn is appended to out/turns.jsonl with the robot's own belief; the
supervisor stamps the same events with ground truth.
"""

import argparse
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import yaml  # noqa: E402
from controller import Robot  # noqa: E402

from robot.config import ControllerConfig  # noqa: E402
from robot.drive import differential_speeds  # noqa: E402
from robot.odometry import Odometry  # noqa: E402
from robot.turn import TurnConfig, TurnController, wrap  # noqa: E402

# Mixed magnitudes and both directions: a bias that only shows on large turns,
# or only turning one way, is a bias worth seeing separately.
SEQUENCE = [90, -90, 180, 45, -45, 90, 90, 90, 90, -180]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path,
                        default=ROOT / "config" / "bot_01.yaml")
    parser.add_argument("--out", type=pathlib.Path, default=ROOT / "out" / "turns.jsonl")
    parser.add_argument("--settle-s", type=float, default=1.0,
                        help="pause between turns, so each starts from rest")
    args = parser.parse_args(argv)

    config = ControllerConfig.from_dict(yaml.safe_load(args.config.read_text()))

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    motors, encoders = {}, {}
    for side in ("left", "right"):
        motor = robot.getDevice("{} wheel motor".format(side))
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors[side] = motor
        encoder = robot.getDevice("{} wheel sensor".format(side))
        encoder.enable(timestep)
        encoders[side] = encoder

    odometry = Odometry(wheel_radius=config.odometry.wheel_radius,
                        track_width=config.odometry.track_width)
    controller = TurnController(TurnConfig())

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("")

    print("track_width={} wheel_radius={}".format(
        config.odometry.track_width, config.odometry.wheel_radius), flush=True)

    index = 0
    pending = None
    resume_at = None

    while robot.step(timestep) != -1:
        now = robot.getTime()
        odometry.update(encoders["left"].getValue(), encoders["right"].getValue())
        heading = odometry.pose.theta

        if not controller.active:
            if pending is not None:
                # Let the chassis come to rest before believing the heading:
                # a reading taken mid-wobble measures the wobble.
                if resume_at is None:
                    resume_at = now + args.settle_s
                    motors["left"].setVelocity(0.0)
                    motors["right"].setVelocity(0.0)
                    continue
                if now < resume_at:
                    continue

                record = dict(pending, believed_end=round(heading, 5), t=round(now, 3))
                with args.out.open("a") as handle:
                    handle.write(json.dumps(record) + "\n")
                print("turn {} of {}: asked {:+.0f} deg, believes {:+.1f} deg".format(
                    record["index"], len(SEQUENCE), record["requested_deg"],
                    math.degrees(wrap(record["believed_end"] - record["believed_start"]))),
                    flush=True)
                pending = None
                resume_at = None

            if index >= len(SEQUENCE):
                break

            requested = SEQUENCE[index]
            pending = {
                "index": index,
                "requested_deg": requested,
                "believed_start": round(heading, 5),
                "t_start": round(now, 3),
            }
            controller.start(heading + math.radians(requested), now)
            index += 1
            continue

        steering, done, timed_out = controller.update(heading, now)
        if timed_out:
            print("turn {} TIMED OUT".format(pending["index"] if pending else "?"), flush=True)
            pending = None

        left, right = differential_speeds(0.0, steering, config.control.max_speed)
        motors["left"].setVelocity(left)
        motors["right"].setVelocity(right)

    for motor in motors.values():
        motor.setVelocity(0.0)
    print("done: {} turns written to {}".format(index, args.out), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
