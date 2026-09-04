"""Spike: drive the line and log what the ground-facing sensors see.

The fleet run showed the robot completing 83% of the oval without the IR array
ever losing the line and without the colour trigger firing, even though it
passed four markers. Either the marker planes are invisible to both sensors or
they are not where the robot drives. This reports raw readings so the answer is
not a guess.

Run as bot_01, with the sim already up.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import yaml  # noqa: E402
from controller import Robot  # noqa: E402

from robot.config import ControllerConfig  # noqa: E402
from robot.marker import BorderDetector  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path,
                        default=pathlib.Path("/project/config/bot_01.yaml"))
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--speed", type=float, default=6.0)
    parser.add_argument("--every", type=float, default=0.5,
                        help="seconds between routine samples")
    args = parser.parse_args(argv)

    config = ControllerConfig.from_dict(yaml.safe_load(args.config.read_text()))
    detector = BorderDetector(config.optics.border_rgb, config.optics.border_tolerance)

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    sensors = []
    for index in range(len(config.sensors.offsets)):
        sensor = robot.getDevice("ir{}".format(index))
        sensor.enable(timestep)
        sensors.append(sensor)

    colour = robot.getDevice("color")
    colour.enable(timestep)
    camera = robot.getDevice("qr")
    camera.enable(timestep)  # left on: this is a diagnostic, not the control loop

    motors = {}
    for side in ("left", "right"):
        motor = robot.getDevice("{} wheel motor".format(side))
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors[side] = motor

    encoders = {}
    for side in ("left", "right"):
        encoder = robot.getDevice("{} wheel sensor".format(side))
        encoder.enable(timestep)
        encoders[side] = encoder

    print("timestep {} ms  border reference rgb={}  tolerance {}".format(
        timestep, config.optics.border_rgb, config.optics.border_tolerance), flush=True)

    started = None
    next_sample = 0.0
    best_distance = float("inf")
    best_rgb = None
    triggers = 0

    while robot.step(timestep) != -1:
        now = robot.getTime()
        if started is None:
            started = now
        elapsed = now - started
        if elapsed >= args.duration:
            break

        motors["left"].setVelocity(args.speed)
        motors["right"].setVelocity(args.speed)

        image = colour.getImage()
        if not image:
            continue
        rgb = (image[2], image[1], image[0])  # BGRA
        distance = detector.distance(rgb)

        if distance < best_distance:
            best_distance = distance
            best_rgb = rgb

        if detector.sees_border(rgb):
            triggers += 1
            if triggers < 40:
                print("  BORDER t={:.2f} rgb={} chroma_d={:.3f}".format(
                    now, rgb, distance), flush=True)

        if elapsed >= next_sample:
            next_sample = elapsed + args.every
            ir = [int(s.getValue()) for s in sensors]
            print("t={:5.1f}  ir={}  color_rgb={}  chroma_d={:.3f}".format(
                now, ir, rgb, distance), flush=True)

    for motor in motors.values():
        motor.setVelocity(0.0)

    # Save what the QR camera actually sees. "a frame arrived" and "the frame
    # shows the marker" are different claims, and only the image settles it.
    camera.saveImage("/project/out/qr_view.png", 100)
    colour.saveImage("/project/out/color_view.png", 100)
    print("saved out/qr_view.png and out/color_view.png", flush=True)

    print("\nclosest the colour sensor ever came to the border: "
          "{:.3f} at rgb={} (threshold {})".format(
              best_distance, best_rgb, config.optics.border_tolerance), flush=True)
    print("border triggers: {}".format(triggers), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
