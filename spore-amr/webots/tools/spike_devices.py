"""Spike: what devices does the robot actually have?

A missing device is silent -- getDevice returns None and the feature that
needed it just never runs -- so this lists what the PROTO really produced.
"""

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from controller import Robot  # noqa: E402


def main():
    robot = Robot()
    count = robot.getNumberOfDevices()
    print("{} devices:".format(count), flush=True)
    for index in range(count):
        device = robot.getDeviceByIndex(index)
        print("  {:<24} {}".format(device.getName(), type(device).__name__), flush=True)

    lidar = robot.getDevice("lidar")
    if lidar is None:
        print("\nNO LIDAR DEVICE", flush=True)
        return 1

    timestep = int(robot.getBasicTimeStep())
    lidar.enable(timestep)
    for _ in range(6):
        robot.step(timestep)
    scan = lidar.getRangeImage()
    print("\nlidar: {} points, max_range {}".format(
        lidar.getHorizontalResolution(), lidar.getMaxRange()), flush=True)
    print("scan: {}".format("none" if not scan else
                            "min %.3f max %.3f" % (min(scan), max(scan))), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
