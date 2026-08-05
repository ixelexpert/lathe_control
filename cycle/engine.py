"""Simultaneous chuck + ballscrew cycle with per-axis start/end delays."""

from __future__ import annotations

import logging
import threading
import time
from enum import Enum
from typing import Callable

from drives.ballscrew_axis import BallscrewAxis
from drives.chuck_axis import ChuckAxis

logger = logging.getLogger(__name__)


class CycleState(str, Enum):
    IDLE = "Idle"
    RUNNING = "Running"
    STOPPING = "Stopping"
    FAULT = "Fault"
    COMPLETE = "Complete"


class CycleEngine:
    def __init__(self, ballscrew: BallscrewAxis, chuck: ChuckAxis) -> None:
        self.ballscrew = ballscrew
        self.chuck = chuck
        self.state = CycleState.IDLE
        self.message = ""
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.on_state: Callable[[CycleState, str], None] | None = None

    def _set_state(self, state: CycleState, message: str = "") -> None:
        self.state = state
        self.message = message
        if self.on_state:
            try:
                self.on_state(state, message)
            except Exception:  # noqa: BLE001
                logger.exception("on_state callback failed")

    @property
    def busy(self) -> bool:
        return self.state in (CycleState.RUNNING, CycleState.STOPPING)

    def start(self) -> None:
        with self._lock:
            if self.busy:
                raise RuntimeError("Cycle already running")
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="CycleEngine", daemon=True)
            self._set_state(CycleState.RUNNING, "Cycle started")
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._set_state(CycleState.STOPPING, "Stopping…")
        try:
            self.chuck.disable()
        except Exception:  # noqa: BLE001
            logger.exception("Chuck stop during cycle abort")
        try:
            self.ballscrew.disable()
        except Exception:  # noqa: BLE001
            logger.exception("Ballscrew stop during cycle abort")

    def estop(self) -> None:
        self.stop()
        self._set_state(CycleState.FAULT, "E-Stop")

    def _wait(self, seconds: float) -> bool:
        """Wait; return False if stopped."""
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if self._stop.is_set():
                return False
            time.sleep(0.02)
        return not self._stop.is_set()

    def _run(self) -> None:
        errors: list[str] = []
        z_move_done = threading.Event()

        def ballscrew_branch() -> None:
            try:
                bp = self.ballscrew.params
                if not self._wait(bp.start_delay_s):
                    return
                if self._stop.is_set():
                    return
                self.ballscrew.move_blocking(bp.distance_mm, stop_event=self._stop)
                z_move_done.set()
                if self._stop.is_set():
                    return
                self._wait(bp.end_delay_s)
            except Exception as exc:  # noqa: BLE001
                logger.exception("Ballscrew branch failed")
                errors.append(f"Ballscrew: {exc}")
                self._stop.set()
                try:
                    self.ballscrew.disable()
                except Exception:  # noqa: BLE001
                    pass
            finally:
                z_move_done.set()

        def chuck_branch() -> None:
            try:
                cp = self.chuck.params
                if not self._wait(cp.start_delay_s):
                    return
                if self._stop.is_set():
                    return
                self.chuck.start()
                if cp.duration_s > 0:
                    if not self._wait(cp.duration_s):
                        self.chuck.stop()
                        return
                    self.chuck.stop()
                else:
                    # Run until ballscrew move completes (before Z end delay)
                    while not z_move_done.is_set():
                        if self._stop.is_set():
                            break
                        time.sleep(0.05)
                    self.chuck.stop()
                if self._stop.is_set():
                    return
                self._wait(cp.end_delay_s)
            except Exception as exc:  # noqa: BLE001
                logger.exception("Chuck branch failed")
                errors.append(f"Chuck: {exc}")
                self._stop.set()
                try:
                    self.chuck.disable()
                except Exception:  # noqa: BLE001
                    pass

        tz = threading.Thread(target=ballscrew_branch, name="Cycle-Z", daemon=True)
        tc = threading.Thread(target=chuck_branch, name="Cycle-C", daemon=True)
        tz.start()
        tc.start()
        tz.join()
        tc.join()

        if errors:
            self._set_state(CycleState.FAULT, "; ".join(errors))
        elif self._stop.is_set() and self.state != CycleState.FAULT:
            self._set_state(CycleState.IDLE, "Cycle aborted")
        else:
            self._set_state(CycleState.COMPLETE, "Cycle complete")
            self._set_state(CycleState.IDLE, "Idle")
