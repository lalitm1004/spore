"""Firmware: the Arduino's job, running as the Webots extern controller.

Owns the sensors and motors and closes the control loop every tick. Talks to
the companion over a serial link, but never depends on it: if the link stalls
or the companion dies, the robot keeps following the line on its own.

This is the only module that imports the Webots API.
"""

import argparse
import errno
import json
import math
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import yaml  # noqa: E402
from controller import Robot  # noqa: E402

from robot.config import ControllerConfig  # noqa: E402
from robot.drive import differential_speeds  # noqa: E402
from robot.events import EventDetector  # noqa: E402
from robot.hal import SampledSensors  # noqa: E402
from robot.line_estimator import LineEstimator  # noqa: E402
from robot.marker import BorderDetector, Crossing, CrossingConfig, MarkerCrossing  # noqa: E402
from robot.obstacle import Obstacle, ObstacleConfig, ObstacleGuard, nearest  # noqa: E402
from robot.odometry import Odometry, Pose  # noqa: E402
from robot.pid import PID  # noqa: E402
from robot.protocol import LineReader, Message, encode  # noqa: E402
from robot.telemetry import TelemetryLog  # noqa: E402


def build_columns(sensor_count):
    raw = ["ir{}".format(i) for i in range(sensor_count)]
    normalised = ["r{}".format(i) for i in range(sensor_count)]
    return ["t"] + raw + normalised + [
        "line_pos", "error", "p", "i", "d", "u", "v_left", "v_right", "lost",
        "distance", "crossing", "node_id", "obstacle", "range_m",
        "border", "bands", "rear_lost", "rear_pos",
    ]


# How far past a tile to hold course before admitting the line is genuinely
# lost. One tile length plus margin: if the lane were there, it would have been
# found by now.
RECOVERY_BUDGET_M = 0.15

# EWMA weight for the steering average held across a crossing.
STEERING_AVERAGE_ALPHA = 0.04

# Index into the LED's `color` list in LineBot.proto.
LED_SEARCHING = 1
LED_BY_KIND = {"PT": 2, "TR": 3, "CH": 4, "PK": 5, "YI": 6}


class Optics:
    """The colour trigger and the QR camera.

    The QR camera is enabled only while a code is actually under it. That is
    the point of the colour trigger -- on hardware it is the difference
    between a camera running continuously and one that wakes a few times a
    minute, and in Webots each enabled camera is a render pass, so it is also
    what keeps a fleet affordable.
    """

    def __init__(self, webots_robot, timestep, config):
        self.config = config
        self.timestep = timestep
        self.detector = BorderDetector(config.border_rgb, config.border_tolerance)
        self.crossing = MarkerCrossing(CrossingConfig(
            tile_length=config.tile_length,
            color_sensor_x=config.color_sensor_x,
            ir_array_x=config.ir_array_x,
        ))

        self.color = webots_robot.getDevice("color")
        self.camera = webots_robot.getDevice("qr")
        self.led = webots_robot.getDevice("status")
        self.available = self.color is not None and self.camera is not None
        if self.available:
            self.color.enable(timestep)  # 1x1: cheap enough to leave running
        if self.led is not None:
            self.led.set(LED_SEARCHING)

        self._camera_on = False
        self._reader = None
        self._reported_crossing = 0
        self.previous_fix = None
        self.last_read = None
        self.reads = 0

    def _reader_or_none(self):
        """Build the decoder lazily so a missing OpenCV degrades to no reads."""
        if self._reader is None:
            try:
                from robot.qr import QrReader

                self._reader = QrReader()
            except Exception as error:  # pragma: no cover - import-time only
                print("qr decoding unavailable: {}".format(error), flush=True)
                self._reader = False
        return self._reader or None

    def sees_border(self):
        image = self.color.getImage()
        if not image:
            return False
        # BGRA, one pixel.
        return self.detector.sees_border((image[2], image[1], image[0]))

    # Heading is not corrected from markers, deliberately.
    #
    # The shared QR schema carries no lane bearing, and the obvious substitute
    # -- the chord between consecutive markers -- is only the lane's direction
    # when the lane between them is straight. On this track's arc, nodes 20 and
    # 30 give a chord of 85 degrees where the lane runs at 133, and feeding
    # that back turned the robot around. In the real warehouse, edges are
    # straight 2 m spans and the chord would be exact; the oval is the outlier,
    # and a correction that is right only on some geometry is worse than none.
    #
    # It costs nothing now anyway: calibrating track_width to 0.0994 m brought
    # wheel-only heading drift to about 0.1 degrees per marker segment, from
    # the 8-10 it was. Odometry heading is good enough to rotate the lever arm
    # with, and the marker still supplies an absolute position fix.

    def fix_from(self, read, distance, heading):
        """Where the read puts the robot's own origin, in facility mm."""
        lever = self.crossing.lever_arm(distance)
        if lever is None:
            return None
        return (read.x_mm - lever * 1000.0 * math.cos(heading),
                read.y_mm - lever * 1000.0 * math.sin(heading))

    def update(self, distance):
        """Advance the crossing state and read the code when it is in view."""
        if not self.available:
            return None

        state = self.crossing.update(distance, self.sees_border())
        wants_camera = self.crossing.should_read(
            distance,
            camera_x=self.config.camera_x,
            footprint=self.config.camera_footprint,
            code_size=self.config.code_size,
        )

        if wants_camera and not self._camera_on:
            # Webots has no frame until the step after enable(), and the
            # Python binding raises on the NULL rather than returning empty.
            # Skip this step rather than guard the read.
            self.camera.enable(self.timestep)
            self._camera_on = True
            return None
        elif not wants_camera and self._camera_on:
            self.camera.disable()
            self._camera_on = False
            return None

        if not wants_camera:
            return None

        reader = self._reader_or_none()
        if reader is None:
            return None

        try:
            image = self.camera.getImage()
        except ValueError:
            return None  # frame not rendered yet
        if not image:
            return None

        from robot.qr import to_gray

        try:
            read = reader.read(to_gray(image, self.camera.getWidth(), self.camera.getHeight()))
        except ValueError:
            return None

        # One report per crossing: the code stays in view for ~17 frames, and
        # the companion wants an arrival, not a stream of duplicates.
        if read is not None and self.crossing.crossings != self._reported_crossing:
            self._reported_crossing = self.crossing.crossings
            self.last_read = read
            self.reads += 1
            if self.led is not None:
                self.led.set(LED_BY_KIND.get(read.kind, LED_SEARCHING))
            return read
        return None


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def write_status(path, name, t, distance, read, fix=None, theta=None,
                 drifted_theta=None):
    """Publish the last marker read for the supervisor to display.

    A small file in the shared project volume rather than a new socket: the
    supervisor runs in its own container and only needs the latest value, so
    a torn read costs nothing and a lost one is replaced 100 mm later.
    """
    payload = {
        "robot": name,
        "t": round(t, 3),
        "distance": round(distance, 4),
        "schema_version": read.schema_version,
        "node_id": read.node_id,
        "name": read.name,
        "region_id": read.region_id,
        "kind": read.kind,
        "x_cm": read.x_cm,
        "y_cm": read.y_cm,
        # Where the robot believes its own origin is, having removed the lever
        # arm between it and the marker. This is the number worth scoring
        # against ground truth; the marker's own position is not.
        "fix_x_mm": round(fix[0], 1) if fix else None,
        "fix_y_mm": round(fix[1], 1) if fix else None,
        "odo_theta": round(theta, 5) if theta is not None else None,
        # Heading as the wheels alone had it, immediately before this fix
        # overwrote it. Comparing this to ground truth is what validates the
        # track-width calibration; comparing `odo_theta` would not, because by
        # then it is the marker's answer, not the wheels'.
        "drifted_theta": round(drifted_theta, 5) if drifted_theta is not None else None,
        "image_rotation": round(read.image_rotation, 5),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload))
        temporary.replace(path)  # atomic, so the reader never sees half a file
    except OSError:
        pass  # a display aid is never worth stalling the control loop


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
    parser.add_argument("--status", type=pathlib.Path, default=None,
                        help="where to publish the last marker read, for the supervisor")
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

    # Rear array, for reversing. Same geometry mirrored, so the same estimator
    # and the same PD gains work -- the loop is only unstable when the sensor
    # trails the direction of travel, and here it never does.
    rear_sensors = []
    for index in range(len(config.sensors.offsets)):
        sensor = webots_robot.getDevice("irb{}".format(index))
        if sensor is None:
            break
        sensor.enable(timestep)
        rear_sensors.append(sensor)
    rear_front_end = SampledSensors(
        adc=config.sensors.adc,
        sample_period_s=config.sensors.sample_period_s,
        latency_s=config.sensors.latency_s,
    ) if rear_sensors else None

    motors = {}
    encoders = {}
    for side in ("left", "right"):
        motor = webots_robot.getDevice("{} wheel motor".format(side))
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors[side] = motor

        encoder = webots_robot.getDevice("{} wheel sensor".format(side))
        if encoder is not None:
            encoder.enable(timestep)
        encoders[side] = encoder

    # Dead reckoning across a marker needs distance travelled, so the encoders
    # stop being decorative here.
    odometry = Odometry(
        wheel_radius=config.odometry.wheel_radius,
        track_width=config.odometry.track_width,
    )
    have_encoders = all(encoder is not None for encoder in encoders.values())

    lidar = webots_robot.getDevice("lidar") if config.lidar.enabled else None
    if lidar is not None:
        lidar.enable(timestep)
    guard = ObstacleGuard(ObstacleConfig(
        stop_m=config.lidar.stop_m,
        clear_m=config.lidar.clear_m,
        decel_s=config.lidar.decel_s,
        pause_s=config.lidar.pause_s,
        accel_s=config.lidar.accel_s,
        borders_to_pass=config.lidar.borders_to_pass,
        max_backoff_m=config.lidar.max_backoff_m,
        departed_m=config.lidar.departed_m,
        backoff_speed=config.lidar.backoff_speed,
    )) if lidar is not None else None

    optics = Optics(webots_robot, timestep, config.optics) if config.optics.enabled else None
    if optics is not None and not optics.available:
        print("{}: no optics devices in the world; markers disabled".format(config.name),
              flush=True)
        optics = None
    if optics is not None and not have_encoders:
        print("{}: no wheel encoders; marker crossing needs odometry, disabling".format(
            config.name), flush=True)
        optics = None

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
    # A second controller for reversing, so neither loop sees the other's
    # derivative history. Softer, because the retreat is slow and a stiff gain
    # at 0.04 m/s just oscillates.
    reverse_pid = PID(kp=control.pid.kp * 0.4, ki=0.0, kd=control.pid.kd * 0.4,
                      output_limit=control.steering_limit)
    detector = EventDetector()
    link = SerialLink(args.link) if args.link else None

    status_path = args.status or ROOT / "out" / "{}.status.json".format(config.name)
    telemetry_path = args.telemetry or ROOT / "out" / "{}.csv".format(config.name)
    log = TelemetryLog(telemetry_path, build_columns(len(sensors)))

    base_speed = control.base_speed
    running = True
    finished = False
    summary = None
    started = None
    lost_since = None
    last_position = 0.0
    last_steering = 0.0
    steering_average = 0.0
    border_now = False
    recovery_from = None
    trip_distance = None

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

        if have_encoders:
            odometry.update(encoders["left"].getValue(), encoders["right"].getValue())
        distance = odometry.pose.distance

        # The reflex reads first and outranks everything below it. A marker
        # read or a route is not a reason to drive into something.
        if guard is not None:
            scan = lidar.getRangeImage() or []
            was_blocked = guard.blocked

            if trip_distance is None and not guard.blocked:
                trip_distance = distance
            border_now = optics.sees_border() if optics is not None else False
            guard.update(
                nearest(scan, config.lidar.max_range),
                now,
                border_now,
                distance - (trip_distance if trip_distance is not None else distance),
                cruise_speed=base_speed,
            )
            if not guard.blocked:
                trip_distance = distance
            if guard.blocked != was_blocked and link is not None:
                link.send(Message(kind="EVT", name="OBSTACLE", fields={
                    "t": round(now, 4),
                    "state": guard.state.value,
                    "range": round(guard.last_range, 4),
                }))

        rear_lost = None
        rear_position = None
        marker_read = optics.update(distance) if optics is not None else None
        if marker_read is not None:
            # An absolute position fix. Heading is left to the odometry: see
            # the note on Optics above for why the marker does not supply one.
            drifted_theta = odometry.pose.theta
            heading = drifted_theta
            fix = optics.fix_from(marker_read, distance, heading)
            if fix is not None:
                odometry.reset(Pose(x=fix[0] / 1000.0, y=fix[1] / 1000.0,
                                    theta=heading,
                                    distance=odometry.pose.distance))
                if guard is None or not guard.blocked:
                    trip_distance = None
            optics.previous_fix = (marker_read.x_mm, marker_read.y_mm)
            print("{}: {}".format(config.name, marker_read.summary), flush=True)
            write_status(status_path, config.name, now, distance, marker_read,
                         fix=fix, theta=heading, drifted_theta=drifted_theta)
            if link is not None:
                link.send(Message(kind="EVT", name="MARKER", fields={
                    "t": round(now, 4),
                    "node": marker_read.node_id,
                    "kind": marker_read.kind,
                    "x_cm": marker_read.x_cm,
                    "y_cm": marker_read.y_cm,
                    "region": marker_read.region_id,
                }))

        # A marker tile covers the line it was following, so for ~115 mm the
        # IR array has nothing to track. Hold the last steering rather than
        # believe the estimator: the robot arrived square by following the
        # line, and markers are laid along the lane, so straight is right.
        crossing_blind = optics is not None and not optics.crossing.line_is_trustworthy(distance)

        counts = front_end.update(now, [sensor.getValue() for sensor in sensors])
        reading = estimator.estimate(counts)

        if optics is not None and optics.crossing.state is Crossing.RECOVERING \
                and not reading.lost:
            optics.crossing.recovered()

        # Coming off a tile the line should be just ahead, so hold course while
        # reacquiring it rather than running the lost-line search. That search
        # is a hard turn at the steering limit -- right when the robot has
        # genuinely wandered off, and a way to spin 180 degrees on the spot
        # when it has not. It turned the robot around after every marker on a
        # curve.
        #
        # Bounded, though: holding indefinitely just drives a circle back onto
        # the same tile, which is how this first presented -- the same marker
        # read over and over.
        if optics is not None and optics.crossing.state is Crossing.RECOVERING:
            if recovery_from is None:
                recovery_from = distance
            if reading.lost and (distance - recovery_from) < RECOVERY_BUDGET_M:
                crossing_blind = True
        else:
            recovery_from = None

        if guard is not None and guard.blocked:
            # The reflex owns the motors. Retreating to a node means driving
            # each wheel back toward the rotation it had there: a differential
            # drive is path-reversible, so replaying both shafts retraces the
            # arc exactly. Reversing in a straight line does not -- the lane
            # curves, and the robot ends up beside the node rather than on it.
            error, steering = 0.0, 0.0
            output = pid.update(error=0.0, dt=dt)
            lost_since = None
            base = guard.speeds(now)

            if guard.state is Obstacle.BACKING and rear_sensors:
                # Steer off the rear array while reversing, so the sensor that
                # leads the direction of travel is the one being followed.
                #
                # The sign is inverted against the forward loop. A point at
                # -L has lateral velocity -L*omega, so moving the rear of the
                # robot toward the line needs the opposite rotation to moving
                # its front toward the line. Following the rear array with the
                # forward sign steers away from the lane, which is what the
                # front array does when reversing and why that failed.
                rear_counts = rear_front_end.update(
                    now, [sensor.getValue() for sensor in rear_sensors])
                rear_reading = estimator.estimate(rear_counts)
                rear_lost = rear_reading.lost
                if not rear_lost:
                    error = rear_reading.position
                    steering = -reverse_pid.update(error=error, dt=dt).u
                    rear_position = error
        elif not running:
            error, steering, base = 0.0, 0.0, 0.0
            output = pid.update(error=0.0, dt=dt)
        elif crossing_blind:
            # Dead reckoning: keep the last good steering and keep moving. The
            # lost-line timer must not run here -- the line is absent by
            # design, not by failure.
            lost_since = None
            error = last_position
            output = pid.update(error=error, dt=dt)
            # Hold the *average* turn rate, not the last instantaneous one.
            # The PD output oscillates around what the curve needs, so a single
            # sample can be near zero on a bend -- and going straight for the
            # 250 mm of a crossing plus reacquisition deviates about 31 mm from
            # a 1 m-radius arc, which is outside a 20 mm lane. The robot came
            # off the tile with no line under it and stopped.
            steering = steering_average
            base = config.optics.crossing_speed or base_speed
        elif reading.lost:
            # The lost-line timeout is for a robot that has wandered off the
            # track, not one part-way through a marker. A crossing plus
            # reacquisition takes about 3 s at the speeds the companion
            # settles on, which is longer than the 2 s timeout -- so leaving
            # the timer running here stopped the robot on a working track.
            if optics is not None and optics.crossing.state is not Crossing.CLEAR:
                lost_since = None
            else:
                lost_since = now if lost_since is None else lost_since
            if lost_since is not None and now - lost_since >= control.lost_line_timeout_s:
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
            last_steering = steering
            # ~0.4 s of history at 62.5 Hz: long enough to average out the PD's
            # oscillation, short enough to still be the curve the robot is on.
            steering_average += (steering - steering_average) * STEERING_AVERAGE_ALPHA
            base = base_speed

        left, right = differential_speeds(base, steering, control.max_speed)
        motors["left"].setVelocity(left)
        motors["right"].setVelocity(right)

        if link is not None:
            # The line is absent by design while crossing a marker tile and
            # while the reflex owns the motors, so neither counts as losing it.
            # Reporting them anyway made the companion read every marker as
            # "going too fast" and throttle: 6.0 -> 3.6 -> 2.16 -> 1.5 rad/s
            # over three tiles, until the robot was crawling.
            # The whole crossing counts, not just the blind stretch: 86 of 90
            # losses in a run were in RECOVERING -- past the tile, reacquiring
            # the line -- and those were enough to throttle the robot to a
            # crawl on their own.
            expected_absence = (
                (optics is not None and optics.crossing.state is not Crossing.CLEAR)
                or (guard is not None and guard.blocked)
            )
            for event in detector.update(now, reading.lost and not expected_absence,
                                         error, steering):
                link.send(event)

        row = {"t": now, "line_pos": reading.position, "error": error,
               "p": output.p, "i": output.i, "d": output.d, "u": steering,
               "v_left": left, "v_right": right, "lost": reading.lost,
               "distance": round(distance, 5),
               "crossing": optics.crossing.state.value if optics else "CLEAR",
               "obstacle": guard.state.value if guard else "CLEAR",
               "border": int(border_now) if guard else "",
               "rear_lost": int(rear_lost) if rear_lost is not None else "",
               "rear_pos": round(rear_position, 5) if rear_position is not None else "",
               "bands": guard.borders_seen if guard else "",
               "range_m": round(guard.last_range, 4) if guard and
                          guard.last_range != float("inf") else "",
               "node_id": optics.last_read.node_id if optics and optics.last_read else ""}
        for index, (value, normalised) in enumerate(zip(counts, reading.normalised)):
            row["ir{}".format(index)] = value
            row["r{}".format(index)] = normalised

        if running:
            log.record(row)
        elif not finished:
            # The run ended when the companion said stop. Keep stepping so the
            # simulation does not stall on a synchronized robot, but stop
            # recording -- stationary zeros would pollute the run's metrics.
            finished = True
            summary = log.close()
            print("{}: {}".format(config.name, summary), flush=True)

    for motor in motors.values():
        motor.setVelocity(0.0)
    if link is not None:
        link.close()

    if not finished:
        summary = log.close()
        print("{}: {}".format(config.name, summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
