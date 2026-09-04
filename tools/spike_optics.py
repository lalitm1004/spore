"""Spike: does Webots still produce camera images under `--no-rendering`?

`--no-rendering` is what took simulator CPU from 907% to 5.65%, so it sets the
affordable fleet size. The QR reader needs a Camera. If the two are mutually
exclusive, that is a design constraint worth discovering in ten minutes rather
than after the reader is written.

Run it against a world whose robot has `hasOptics TRUE`, once with rendering on
and once with it off, and compare:

    RENDERING=on  docker compose up -d sim
    /usr/local/webots/webots-controller --protocol=tcp --ip-address=localhost \
        --port=1234 --robot-name=bot_01 tools/spike_optics.py

Exits non-zero if any enabled camera never delivers a frame, so it works as a
gate in CI as well as by eye.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from controller import Robot  # noqa: E402


def describe(camera):
    width, height = camera.getWidth(), camera.getHeight()
    return "{}x{} fov={:.3f}".format(width, height, camera.getFov())


def probe(camera):
    """Return (ok, detail) for one camera's current frame.

    Webots hands back BGRA, so a healthy frame is exactly width*height*4 bytes.
    A short buffer is as much a failure as a missing one -- it means the frame
    was not rendered, only allocated.
    """
    image = camera.getImage()
    if not image:
        return False, "no image (getImage returned {!r})".format(image)

    width, height = camera.getWidth(), camera.getHeight()
    expected = width * height * 4
    if len(image) != expected:
        return False, "short buffer: {} bytes, expected {}".format(len(image), expected)

    # Centre pixel, BGRA.
    index = ((height // 2) * width + (width // 2)) * 4
    blue, green, red = image[index], image[index + 1], image[index + 2]
    return True, "ok, {} bytes, centre rgb=({}, {}, {})".format(len(image), red, green, blue)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=40,
                        help="control steps to run before giving up on a frame")
    parser.add_argument("--cameras", nargs="*", default=["color", "qr"])
    args = parser.parse_args(argv)

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    cameras = {}
    for name in args.cameras:
        device = robot.getDevice(name)
        if device is None:
            print("MISSING  {}: no such device in the PROTO".format(name), flush=True)
            continue
        device.enable(timestep)
        cameras[name] = device
        print("enabled  {}: {}".format(name, describe(device)), flush=True)

    if not cameras:
        print("VERDICT: no cameras found -- is hasOptics TRUE?", flush=True)
        return 2

    # A camera needs at least one step after enable() before a frame exists;
    # keep stepping so a slow first render is not misread as no render at all.
    first_frame = {name: None for name in cameras}
    for step in range(args.steps):
        if robot.step(timestep) == -1:
            break
        for name, camera in cameras.items():
            if first_frame[name] is None:
                ok, detail = probe(camera)
                if ok:
                    first_frame[name] = (step, detail)

    failed = []
    for name in cameras:
        if first_frame[name] is None:
            _, detail = probe(cameras[name])
            print("FAIL     {}: {} after {} steps".format(name, detail, args.steps), flush=True)
            failed.append(name)
        else:
            step, detail = first_frame[name]
            print("PASS     {}: first frame at step {} -- {}".format(name, step, detail), flush=True)

    if failed:
        print("VERDICT: {} produced no frames. If this run had --no-rendering, "
              "the QR camera and the CPU saving are mutually exclusive.".format(
                  ", ".join(failed)), flush=True)
        return 1

    print("VERDICT: all cameras delivered frames.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
