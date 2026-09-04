"""Firmware: the Arduino's job, running as the Webots extern controller.

Owns the sensors and motors and closes the control loop every tick. Talks to
the companion over a serial link, but never depends on it: if the link stalls
or the companion dies, the robot keeps following the line on its own.

This is the only module that imports the Webots API.
"""

import argparse
import errno
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import yaml  # noqa: E402
from controller import Robot  # noqa: E402

from robot.config import ControllerConfig  # noqa: E402
from robot.drive import differential_speeds  # noqa: E402
from robot.events import EventDetector  # noqa: E402
from robot.hal import SampledSensors  # noqa: E402
from robot.line_estimator import LineEstimator  # noqa: E402
from robot.pid import PID  # noqa: E402
from robot.protocol import LineReader, encode  # noqa: E402
from robot.telemetry import TelemetryLog  # noqa: E402


def build_columns(sensor_count):
    raw = ["ir{}".format(i) for i in range(sensor_count)]
    normalised = ["r{}".format(i) for i in range(sensor_count)]
    return ["t"] + raw + normalised + [
        "line_pos", "error", "p", "i", "d", "u", "v_left", "v_right", "lost"
    ]


class SerialLink:
    """Non-blocking serial link. The tight loop must never wait on the Pi."""

    def __init__(self, path):
        self.fd = os.open(str(path), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.reader = LineReader()

    def poll(self):
        try:
            chunk = os.read(self.fd, 4096)
        except OSError as error:
            if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return []
            raise
        return list(self.reader.feed(chunk))

    def send(self, message):
        try:
            os.write(self.fd, encode(message))
        except OSError as error:
            # A full TX buffer drops telemetry rather than stalling control,
            # which is what real firmware does too.
            if error.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                raise

    def close(self):
        os.close(self.fd)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--telemetry", type=pathlib.Path, default=None)
    parser.add_argument("--link", type=pathlib.Path, default=None,
                        help="serial device shared with the companion")
    parser.add_argument("--duration", type=float, default=None)
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

    front_end = SampledSensors(
        adc=config.sensors.adc,
        sample_period_s=config.sensors.sample_period_s,
        latency_s=config.sensors.latency_s,
    )
    estimator = LineEstimator(
        offsets=config.sensors.offsets,
        white_ref=config.sensors.white_ref,
        black_ref=config.sensors.black_ref,
        min_confidence=config.sensors.min_confidence,
    )
    pid = PID(kp=control.pid.kp, ki=control.pid.ki, kd=control.pid.kd,
              output_limit=control.steering_limit)
    detector = EventDetector()
    link = SerialLink(args.link) if args.link else None

    telemetry_path = args.telemetry or pathlib.Path("out/{}.csv".format(config.name))
    log = TelemetryLog(telemetry_path, build_columns(len(sensors)))

    base_speed = control.base_speed
    running = True
    started = None
    lost_since = None
    last_position = 0.0

    while webots_robot.step(timestep) != -1:
        now = webots_robot.getTime()
        if started is None:
            started = now
        if args.duration is not None and now - started >= args.duration:
            break

        if link is not None:
            for command in link.poll():
                if command.name == "SET_SPEED":
                    base_speed = float(command.fields.get("value", base_speed))
                elif command.name == "SET_GAINS":
                    pid.kp = float(command.fields.get("kp", pid.kp))
                    pid.ki = float(command.fields.get("ki", pid.ki))
                    pid.kd = float(command.fields.get("kd", pid.kd))
                elif command.name == "STOP":
                    running = False
                elif command.name == "START":
                    running = True

        counts = front_end.update(now, [sensor.getValue() for sensor in sensors])
        reading = estimator.estimate(counts)

        if not running:
            error, steering, base = 0.0, 0.0, 0.0
            output = pid.update(error=0.0, dt=dt)
        elif reading.lost:
            lost_since = now if lost_since is None else lost_since
            if now - lost_since >= control.lost_line_timeout_s:
                print("{}: line lost for {:.1f}s, stopping".format(
                    config.name, control.lost_line_timeout_s), flush=True)
                break
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
            base = base_speed

        left, right = differential_speeds(base, steering, control.max_speed)
        motors["left"].setVelocity(left)
        motors["right"].setVelocity(right)

        if link is not None:
            for event in detector.update(now, reading.lost, error, steering):
                link.send(event)

        row = {"t": now, "line_pos": reading.position, "error": error,
               "p": output.p, "i": output.i, "d": output.d, "u": steering,
               "v_left": left, "v_right": right, "lost": reading.lost}
        for index, (value, normalised) in enumerate(zip(counts, reading.normalised)):
            row["ir{}".format(index)] = value
            row["r{}".format(index)] = normalised
        log.record(row)

    for motor in motors.values():
        motor.setVelocity(0.0)
    if link is not None:
        link.close()

    summary = log.close()
    print("{}: {}".format(config.name, summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
