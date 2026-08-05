"""Ballscrew axis using the proven A6 multi-segment relative move handshake."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .modbus_bus import ModbusBus
from . import units

logger = logging.getLogger(__name__)

C00_00 = 0x0000
C03_00 = 0x0300
C04_00 = 0x0400
C04_01 = 0x0401
C04_11 = 0x0411
C11_00 = 0x1100
C11_01 = 0x1101
C11_03 = 0x1103
C11_04 = 0x1104
C11_06 = 0x1106
C11_08 = 0x1108
C11_0A = 0x110A
C11_0C = 0x110C
C11_0E = 0x110E
C12_00 = 0x1200
U40_16 = 0x4016
U41_01 = 0x4101

DI1_PROFILE_TRIGGER = 19


@dataclass
class BallscrewParams:
    slave_id: int = 1
    baud: int = 115200
    gear_ratio: float = 1.0
    pitch_mm: float = 5.0
    pulses_per_rev: int = 10000
    axis_speed_mm_s: float = 10.0
    acceleration_ms: float = 200.0
    deceleration_ms: float = 200.0
    distance_mm: float = 10.0
    home_position_mm: float = 0.0
    soft_min_mm: float = -500.0
    soft_max_mm: float = 500.0
    start_delay_s: float = 0.0
    end_delay_s: float = 0.0
    invert_direction: bool = False
    max_axis_mm_s: float = 200.0


@dataclass
class BallscrewStatus:
    enabled: bool = False
    moving: bool = False
    motor_rpm: float = 0.0
    axis_speed_mm_s: float = 0.0
    position_mm: float = 0.0
    encoder_pulses: int = 0
    fault: str = ""
    last_error: str = ""


class BallscrewAxis:
    def __init__(self, bus: ModbusBus, params: BallscrewParams | None = None) -> None:
        self.bus = bus
        self.params = params or BallscrewParams()
        self.status = BallscrewStatus()
        self._home_offset_pulses: int = 0
        self._configured = False

    def _conv_kwargs(self) -> dict:
        return {
            "pitch_mm": self.params.pitch_mm,
            "gear_ratio": self.params.gear_ratio,
            "pulses_per_rev": self.params.pulses_per_rev,
        }

    @property
    def motor_speed_rpm(self) -> float:
        return units.ballscrew_axis_speed_to_motor_rpm(
            abs(self.params.axis_speed_mm_s),
            pitch_mm=self.params.pitch_mm,
            gear_ratio=self.params.gear_ratio,
        )

    def set_axis_speed(self, mm_s: float) -> None:
        self.params.axis_speed_mm_s = units.clamp(
            mm_s, -self.params.max_axis_mm_s, self.params.max_axis_mm_s
        )

    def set_motor_speed(self, motor_rpm: float) -> None:
        mm_s = units.ballscrew_motor_rpm_to_axis_speed(
            motor_rpm,
            pitch_mm=self.params.pitch_mm,
            gear_ratio=self.params.gear_ratio,
        )
        self.set_axis_speed(mm_s)

    def estimated_duration_s(self) -> float:
        return units.estimate_move_duration_s(
            self.params.distance_mm,
            self.params.axis_speed_mm_s,
            self.params.acceleration_ms,
            self.params.deceleration_ms,
        )

    def _distance_pulses(self, distance_mm: float) -> int:
        signed = float(distance_mm)
        if self.params.invert_direction:
            signed = -signed
        return units.ballscrew_mm_to_pulses(signed, **self._conv_kwargs())

    def configure(self) -> None:
        # Full config happens per-move (must be disabled first). Mark ready.
        self._configured = True
        logger.info("Ballscrew axis ready (slave %s)", self.params.slave_id)

    def apply_motion_params(self, distance_mm: float | None = None) -> int:
        dist = self.params.distance_mm if distance_mm is None else float(distance_mm)
        return self._send_move_details(self._distance_pulses(dist), max(1, int(round(self.motor_speed_rpm))))

    def _send_move_details(self, displacement: int, rpm: int) -> int:
        """Proven handshake: disable, load relative multi-segment move, confirm."""
        sid = self.params.slave_id
        bus = self.bus
        bus.write_u16(sid, C04_01, 0)
        bus.write_u16(sid, C04_11, 0)
        time.sleep(0.05)
        bus.write_u16(sid, C00_00, 0)  # position
        bus.write_u16(sid, C12_00, 0)  # speed profile off
        bus.write_u16(sid, C03_00, 1)  # multi-segment
        bus.write_u16(sid, C11_00, 0)  # single
        bus.write_u16(sid, C11_01, 1)  # RELATIVE
        bus.write_u16(sid, C11_03, 1)
        bus.write_u16(sid, C11_04, 1)
        bus.write_i32(sid, C11_06, displacement)  # low-word-first signed
        bus.write_u16(sid, C11_08, rpm)
        bus.write_u32(sid, C11_0A, int(round(self.params.acceleration_ms)))
        bus.write_u32(sid, C11_0C, int(round(self.params.deceleration_ms)))
        bus.write_u32(sid, C11_0E, 0)
        bus.write_u16(sid, C04_00, DI1_PROFILE_TRIGGER)

        time.sleep(0.02)
        got_src = bus.read_u16(sid, C03_00)
        got_rel = bus.read_u16(sid, C11_01)
        got_fn = bus.read_u16(sid, C04_00)
        got_delta = bus.read_i32(sid, C11_06)
        got_rpm = bus.read_u16(sid, C11_08)
        if got_src != 1 or got_rel != 1 or got_fn != DI1_PROFILE_TRIGGER:
            raise RuntimeError(
                f"drive reject config: C03.00={got_src} C11.01={got_rel} C04.00={got_fn}"
            )
        if got_delta != displacement:
            raise RuntimeError(
                f"drive C11.06 mismatch: wrote {displacement}, read {got_delta}"
            )
        if got_rpm != rpm:
            raise RuntimeError(f"drive C11.08 mismatch: wrote {rpm}, read {got_rpm}")
        logger.info("Ballscrew armed: delta=%s pulses rpm=%s", displacement, rpm)
        return displacement

    def enable(self) -> None:
        self.bus.write_u16(self.params.slave_id, C04_11, 1)
        self.status.enabled = True

    def disable(self) -> None:
        try:
            sid = self.params.slave_id
            self.bus.write_u16(sid, C04_01, 0)
            self.bus.write_u16(sid, C04_11, 0)
        except Exception as exc:  # noqa: BLE001
            self.status.last_error = str(exc)
            logger.exception("Ballscrew disable failed")
        self.status.enabled = False
        self.status.moving = False

    def home_here(self) -> None:
        try:
            self.bus.write_u16(self.params.slave_id, C04_01, 0)
        except Exception:  # noqa: BLE001
            pass
        self.poll()
        self._home_offset_pulses = self.status.encoder_pulses
        self.params.home_position_mm = 0.0
        self.poll()
        logger.info(
            "Home Here (no motion): offset=%s pos_mm=%.3f",
            self._home_offset_pulses,
            self.status.position_mm,
        )

    def _check_soft_limits(self, distance_mm: float) -> None:
        self.poll()
        target = self.status.position_mm + distance_mm
        if target < self.params.soft_min_mm - 1e-6 or target > self.params.soft_max_mm + 1e-6:
            raise ValueError(
                f"Move to {target:.2f} mm exceeds soft limits "
                f"[{self.params.soft_min_mm}, {self.params.soft_max_mm}]"
            )

    def start_move(self, distance_mm: float | None = None) -> None:
        dist = self.params.distance_mm if distance_mm is None else float(distance_mm)
        if abs(dist) < 1e-9:
            raise ValueError("Distance is 0")
        self.params.distance_mm = dist
        self._check_soft_limits(dist)

        sid = self.params.slave_id
        pulses = self._distance_pulses(dist)
        rpm = max(1, int(round(self.motor_speed_rpm)))
        self._send_move_details(pulses, rpm)

        # Enable, settle, then DI1 rising edge (held high during motion)
        self.bus.write_u16(sid, C04_11, 1)
        time.sleep(0.1)
        if self.bus.read_u16(sid, C04_11) != 1:
            raise RuntimeError("servo did not enable (C04.11 != 1)")
        time.sleep(0.15)
        self.bus.write_u16(sid, C04_01, 0)
        time.sleep(0.08)
        self.bus.write_u16(sid, C04_01, 1)
        self.status.enabled = True
        self.status.moving = True
        self._configured = True

    def wait_until_stopped(self, *, timeout_s: float, stop_event=None) -> None:
        sid = self.params.slave_id
        start = self.bus.read_i32(sid, U40_16)
        target_delta = self._distance_pulses(self.params.distance_mm)
        target = start + target_delta
        tol = max(4, abs(target_delta) // 200)
        deadline = time.monotonic() + timeout_s
        start_deadline = time.monotonic() + 3.0
        moved = False
        settled = 0
        last = start
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                break
            pos = self.bus.read_i32(sid, U40_16)
            if abs(pos - start) > max(4, tol // 2):
                moved = True
            if abs(pos - target) <= tol:
                settled += 1
                if settled >= 4:
                    break
            else:
                settled = 0
            if not moved and time.monotonic() > start_deadline:
                # retry one trigger edge
                self.bus.write_u16(sid, C04_01, 0)
                time.sleep(0.08)
                self.bus.write_u16(sid, C04_01, 1)
                start_deadline = time.monotonic() + 3.0
            last = pos
            time.sleep(0.05)
        try:
            self.bus.write_u16(sid, C04_01, 0)
        except Exception:  # noqa: BLE001
            pass
        self.status.moving = False
        self.poll()
        logger.info(
            "Ballscrew stop: start=%s last=%s target=%s moved=%s",
            start,
            last,
            target,
            moved,
        )
        if not moved:
            raise RuntimeError(
                f"ballscrew did not move (start={start} last={last} cmd_delta={target_delta})"
            )

    def move_blocking(self, distance_mm: float | None = None, *, stop_event=None) -> None:
        dist = self.params.distance_mm if distance_mm is None else distance_mm
        self.start_move(dist)
        timeout = max(8.0, self.estimated_duration_s() * 3.0 + 3.0)
        self.wait_until_stopped(timeout_s=timeout, stop_event=stop_event)

    def poll(self) -> BallscrewStatus:
        try:
            sid = self.params.slave_id
            enc = self.bus.read_i32(sid, U40_16)
            self.status.encoder_pulses = enc
            rel = enc - self._home_offset_pulses
            mm = units.ballscrew_pulses_to_mm(rel, **self._conv_kwargs())
            if self.params.invert_direction:
                mm = -mm
            self.status.position_mm = mm + self.params.home_position_mm
            try:
                fault = self.bus.read_u16(sid, U41_01)
                self.status.fault = "" if fault == 0 else f"Er.{fault}"
            except Exception:  # noqa: BLE001
                pass
            self.status.last_error = ""
        except Exception as exc:  # noqa: BLE001
            self.status.last_error = str(exc)
            logger.debug("Ballscrew poll failed: %s", exc)
        return self.status
