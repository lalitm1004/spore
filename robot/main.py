"""Webots adapter for the line follower.

This is the only module that imports the Webots API. Everything it calls is
pure and host-testable, so replacing this file is all that a move to ROS 2 or
real hardware requires.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import yaml  # noqa: E402
from controller import Robot  # noqa: E402

from robot.config import ControllerConfig  # noqa: E402
from robot.drive import differential_speeds  # noqa: E402
from robot.line_estimator import LineEstimator  # noqa: E402
from robot.pid import PID  # noqa: E402
from robot.telemetry import TelemetryLog  # noqa: E402


def build_columns(sensor_count: int):
    raw = ["ir{}".format(i) for i in range(sensor_count)]
    normalised = ["r{}".format(i) for i in range(sensor_count)]
    return ["t"] + raw + normalised + [
        "line_pos", "error", "p", "i", "d", "u", "v_left", "v_right", "lost"
    ]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--telemetry", type=pathlib.Path, default=None)
    parser.add_argument("--duration", type=float, default=None,
                        help="simulated seconds to run before exiting")
    args = parser.parse_args(argv)

    config = ControllerConfig.from_dict(yaml.safe_load(args.config.read_text()))
    control = config.control

    webots_robot = Robot()
    timestep = int(webots_robot.getBasicTimeStep())
    dt = timestep / 1000.0

    sensors = []
    for index in range(len(config.sensors.offsets)):
        sensor = webots_robot.getDevice("ir{}".format(index))
        sensor.enable(timestep)
        sensors.append(sensor)

    motors = {}
    for side in ("left", "right"):
        motor = webots_robot.getDevice("{} wheel motor".format(side))
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors[side] = motor

    estimator = LineEstimator(
        offsets=config.sensors.offsets,
        white_ref=config.sensors.white_ref,
        black_ref=config.sensors.black_ref,
        min_confidence=config.sensors.min_confidence,
    )
    pid = PID(
        kp=control.pid.kp,
        ki=control.pid.ki,
        kd=control.pid.kd,
        output_limit=control.steering_limit,
    )

    # Relative to the working directory, which is the mounted project root.
    telemetry_path = args.telemetry or pathlib.Path("out/{}.csv".format(config.name))
    log = TelemetryLog(telemetry_path, build_columns(len(sensors)))

    started = None
    lost_since = None
    last_position = 0.0

    while webots_robot.step(timestep) != -1:
        now = webots_robot.getTime()
        if started is None:
            started = now
        if args.duration is not None and now - started >= args.duration:
            break

        raw = [sensor.getValue() for sensor in sensors]
        reading = estimator.estimate(raw)

        if reading.lost:
            lost_since = now if lost_since is None else lost_since
            if now - lost_since >= control.lost_line_timeout_s:
                print("{}: line lost for {:.1f}s, stopping".format(
                    config.name, control.lost_line_timeout_s), flush=True)
                break
            # Turn back toward wherever the line was last seen.
            error = last_position
            output = pid.update(error=error, dt=dt)
            steering = control.steering_limit * (1.0 if last_position >= 0 else -1.0)
            base = 0.0
        else:
            lost_since = None
            last_position = reading.position
            error = reading.position
            output = pid.update(error=error, dt=dt)
            steering = output.u
            base = control.base_speed

        left, right = differential_speeds(base, steering, control.max_speed)
        motors["left"].setVelocity(left)
        motors["right"].setVelocity(right)

        row = {"t": now, "line_pos": reading.position, "error": error,
               "p": output.p, "i": output.i, "d": output.d, "u": steering,
               "v_left": left, "v_right": right, "lost": reading.lost}
        for index, (value, normalised) in enumerate(zip(raw, reading.normalised)):
            row["ir{}".format(index)] = value
            row["r{}".format(index)] = normalised
        log.record(row)

    for motor in motors.values():
        motor.setVelocity(0.0)

    summary = log.close()
    print("{}: {}".format(config.name, summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
