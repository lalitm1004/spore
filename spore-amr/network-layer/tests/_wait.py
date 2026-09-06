"""One `wait_until` for every tier.

Its own module rather than a conftest function because the container harness
imports it and conftest imports the container harness's fixtures; a cycle, and
one that only works if the function happens to be defined first.
"""
from __future__ import annotations

import time

import grpc


def wait_until(pred, timeout: float = 10.0, step: float = 0.05, what: str = "") -> bool:
    """Poll `pred()` until true or `timeout` elapses.

    `what` names the thing being waited for and is reported on failure; without
    it a timeout reads as a bare `assert False` with nothing to go on. A gRPC
    error while polling is not a failure -- a container still starting, paused,
    or partitioned answers that way, and the point is to keep asking.

    The one implementation. The container suite used to carry its own, which
    shadowed this one and differed only in its poll step; it now binds that step
    with `functools.partial`.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if pred():
                return True
        except grpc.RpcError:
            pass
        time.sleep(step)
    if what:
        print("timed out after {:.0f}s waiting for {}".format(timeout, what), flush=True)
    try:
        return bool(pred())
    except grpc.RpcError:
        return False
