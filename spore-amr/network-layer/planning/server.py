"""The robot link: a unix socket the companion asks questions on.

WHAT
    `RobotLink` — accepts the companion's connection, reads newline-delimited
    JSON `Query` messages, and answers each with a `Decision`.

WHERE
    Started by `bot.Bot.start()` and stopped on shutdown, on its own thread.
    The companion dials the path in `ROBOT_SOCKET`; in the Webots fleet that is
    set per robot by `spore-amr/webots/tools/gen_fleet.py`, and each robot
    container runs one of these bots beside its companion.

WHY
    The robot is blind. It arrives at a QR node, works out which turns
    physically exist from its own copy of the map, asks which to take, and
    *blocks* until it hears back. If it hears nothing it sits there — it only
    asks again on reaching the next node, and it will not reach one.

    So this loop has one hard rule: **every query gets an answer**. A malformed
    query, a planner that raised, a robot standing somewhere our map has never
    heard of — all of them get a Decision. Even a wrong turn is recoverable at
    the next node; silence is not recoverable at all.

HOW
    One long-lived connection, not one per question. The companion connects
    once and keeps it; a robot asks roughly every two seconds for its whole
    shift, and a fresh socket per question would be pure overhead on hardware
    that has none to spare. A companion that dies simply closes, and the next
    one connects to the same listener.

    Answering happens on this thread, not the run loop's, so a slow plan delays
    one robot rather than the whole bot. Planning is a few milliseconds against
    a five-second timeout, which is the margin that makes that safe.
"""

from __future__ import annotations

import config

import json
import logging
import os
import socket
import threading
from collections.abc import Callable
from pathlib import Path

from planning.decide import Decision, DecisionKind, Query

log = logging.getLogger(__name__)

Router = Callable[[Query], Decision]


class RobotLink:
    """Serves `Query` -> `Decision` on a unix socket for one robot."""

    def __init__(
        self,
        path: str | Path,
        router: Router,
        *,
        bot_id: int = 0,
        accept_timeout_s: float = 1.0,
    ) -> None:
        self.path = Path(path)
        self._router = router
        self._bot_id = bot_id
        # Bounded so the accept loop notices `stop()` promptly instead of
        # sitting in a blocking accept until a robot happens to connect.
        self._accept_timeout_s = accept_timeout_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._server: socket.socket | None = None
        self.answered = 0

    # ---- Lifecycle -----------------------------------------------------------

    def start(self) -> bool:
        self.stop()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                # A socket file left by a previous run. Removing it is safe: a
                # live listener would have been ours, and we are replacing it.
                self.path.unlink()
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            server.settimeout(self._accept_timeout_s)
            server.bind(str(self.path))
            server.listen(1)
            os.chmod(str(self.path), 0o777)
        except OSError as error:
            # A bot with no robot attached is still a useful fleet member: it
            # heartbeats, votes and holds a region. Refusing to boot over a
            # socket would take out the membership layer too.
            log.warning("bot-%d: cannot serve the robot link at %s: %s",
                        self._bot_id, self.path, error)
            return False

        self._server = server
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._serve, args=(self._stop,), daemon=True,
            name=f"robot-link-{self._bot_id}",
        )
        self._thread.start()
        log.info("bot-%d: robot link listening on %s", self._bot_id, self.path)
        return True

    def stop(self) -> None:
        self._stop.set()
        server, self._server = self._server, None
        if server is not None:
            server.close()
        thread, self._thread = self._thread, None
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=config.T_THREAD_JOIN)
        if self.path.exists():
            try:
                self.path.unlink()
            except OSError:
                # Best effort. A socket file we cannot remove is tidiness, not
                # correctness -- the next start() unlinks it anyway -- and
                # raising here would break an otherwise clean shutdown.
                pass

    # ---- The loop ------------------------------------------------------------

    def _serve(self, stop: threading.Event) -> None:
        while not stop.is_set():
            server = self._server
            if server is None:
                return
            try:
                connection, _ = server.accept()
            except (TimeoutError, socket.timeout):
                continue
            except OSError:
                return  # the listener was closed under us: we are stopping
            with connection:
                self._converse(connection, stop)

    def _converse(self, connection: socket.socket, stop: threading.Event) -> None:
        """Answer one companion until it goes away."""
        connection.settimeout(self._accept_timeout_s)
        buffer = b""
        while not stop.is_set():
            try:
                chunk = connection.recv(4096)
            except (TimeoutError, socket.timeout):
                continue  # the robot is driving; nothing to answer yet
            except OSError:
                return
            if not chunk:
                return  # the companion closed: not an error, just a shift ending

            buffer += chunk
            while b"\n" in buffer:
                line, _, buffer = buffer.partition(b"\n")
                text = line.decode("utf-8", "replace").strip()
                if not text:
                    continue
                reply = self._answer(text)
                if reply is None:
                    continue
                try:
                    connection.sendall((reply.to_json() + "\n").encode("utf-8"))
                    self.answered += 1
                except OSError:
                    return

    def _answer(self, text: str) -> Decision | None:
        """Route one query, refusing to fail silently.

        A query we cannot even parse has no `query_id` to answer against, and a
        reply carrying the wrong one is discarded by the robot anyway — that is
        the only case where saying nothing is right, and it is logged.
        """
        try:
            query = Query.from_json(text)
        except (ValueError, KeyError, TypeError) as error:
            log.warning("bot-%d: unparseable query (%s): %s",
                        self._bot_id, error, text[:120])
            return None

        try:
            return self._router(query)
        except Exception:
            # Whatever went wrong upstream, the robot is still standing at a
            # node waiting. Hold it briefly and let it ask again rather than
            # leaving it there for the rest of the shift.
            log.exception("bot-%d: planning failed for node %d",
                          self._bot_id, query.node_id)
            return Decision(
                query_id=query.query_id,
                kind=DecisionKind.WAIT,
                hold_ms=1000,
                because="planner error",
            )


def parse_decision(text: str) -> dict:
    """Decode a Decision reply — for tests and tooling on the far side."""
    return json.loads(text)
