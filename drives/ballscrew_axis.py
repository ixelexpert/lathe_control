"""Ballscrew axis — A6-RS multi-position relative moves over Modbus."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .modbus_bus import ModbusBus
from . import units

logger = logging.getLogger(__name__)

# Registers
C00_00 = 0x0000
C00_02 = 0x0002  # pulses per rev (electronic gear related)
C03_00 = 0x0300  # position reference source
C04_00 = 0x0400  # DI1 function
C04_01 = 0x0401  # DI1 virtual level
C04_11 = 0x0411  # enable
C11_00 = 0x1100
C11_01 = 0x1101  # 0 absolute / 1 relative
C11_03 = 0x1103  # start group
C11_04 = 0x1104  # end group
C11_06 = 0x1106  # group1 displacement (U32/I32)
C11_08 = 0x1108  # group1 speed rpm
C11_0A = 0x110A  # group1 accel ms (U32)
C11_0C = 0x110C  # group1 decel ms (U32)
C11_0E = 0x110E  # group1 post delay ms (U32)
U40_01 = 0x4001  # motor speed
U40_16 = 0x4016  # absolute position


@dataclass
class BallscrewParams:
    slave_id: int = 1
    baud: int = 115200
    gear_ratio: float = 1.0
    pitch_mm: float = 10.0
    pulses_per_rev: int = 10000
    axis_speed_mm_s: float = 20.0
    acceleration_ms: float = 200.0
    deceleration_ms: float = 200.0
    distance_mm: float = 50.0
    home_position_mm: float = 0.0
    soft_min_mm: float = 0.0
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
            self.params.axis_speed_mm_s,
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

    def configure(self) -> None:
        sid = self.params.slave_id
        self.bus.write_u16(sid, C00_00, 0)  # position mode
        self.bus.write_u16(sid, C03_00, 1)  # multi-position reference
        self.bus.write_u16(sid, C11_00, 0)  # single operation
        self.bus.write_u16(sid, C11_01, 1)  # relative positioning
        self.bus.write_u16(sid, C11_03, 1)
        self.bus.write_u16(sid, C11_04, 1)
        self.bus.write_u16(sid, C04_00, 19)  # DI1 = position profile trigger (FunIN.19)
        self.apply_motion_params()
        self._configured = True
        logger.info("Ballscrew axis configured (slave %s)", sid)

    def apply_motion_params(self) -> None:
        sid = self.params.slave_id
        pulses = self._distance_pulses(self.params.distance_mm)
        rpm = int(round(abs(self.motor_speed_rpm)))
        rpm = max(1, rpm)
        self.bus.write_i32(sid, C11_06, pulses)
        self.bus.write_u16(sid, C11_08, rpm)
        self.bus.write_u32(sid, C11_0A, int(round(self.params.acceleration_ms)))
        self.bus.write_u32(sid, C11_0C, int(round(self.params.deceleration_ms)))
        self.bus.write_u32(sid, C11_0E, 0)

    def _distance_pulses(self, distance_mm: float) -> int:
        signed = distance_mm
        if self.params.invert_direction:
            signed = -signed
        return units.ballscrew_mm_to_pulses(signed, **self._conv_kwargs())

    def enable(self) -> None:
        if not self._configured:
            self.configure()
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
        """Set software zero at current encoder position (GUI Home Here)."""
        self.poll()
        self._home_offset_pulses = self.status.encoder_pulses
        self.params.home_position_mm = 0.0
        self.poll()
        logger.info(
            "Home Here: offset_pulses=%s position_mm=%.3f",
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
        self.params.distance_mm = dist
        self._check_soft_limits(dist)
        if not self._configured:
            self.configure()
        else:
            self.apply_motion_params()

        sid = self.params.slave_id
        self.bus.write_u16(sid, C04_11, 1)
        # Edge-trigger profile: ensure low then high
        self.bus.write_u16(sid, C04_01, 0)
        time.sleep(0.05)
        self.bus.write_u16(sid, C04_01, 1)
        self.status.enabled = True
        self.status.moving = True

    def wait_until_stopped(self, *, timeout_s: float, stop_event=None) -> None:
        """Wait until motor speed near zero after a move, or timeout."""
        deadline = time.monotonic() + timeout_s
        # Brief settle so motion has started
        time.sleep(0.15)
        idle_count = 0
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                break
            self.poll()
            if abs(self.status.motor_rpm) < 2.0:
                idle_count += 1
                if idle_count >= 4:
                    break
            else:
                idle_count = 0
            time.sleep(0.05)
        self.status.moving = False
        # Drop trigger level
        try:
            self.bus.write_u16(self.params.slave_id, C04_01, 0)
        except Exception:  # noqa: BLE001
            pass

    def move_blocking(self, distance_mm: float | None = None, *, stop_event=None) -> None:
        dist = self.params.distance_mm if distance_mm is None else distance_mm
        self.start_move(dist)
        timeout = max(5.0, self.estimated_duration_s() * 2.5 + 2.0)
        self.wait_until_stopped(timeout_s=timeout, stop_event=stop_event)

    def poll(self) -> BallscrewStatus:
        try:
            sid = self.params.slave_id
            enc = self.bus.read_i32(sid, U40_16)
            speed = self.bus.read_i32(sid, U40_01)
            self.status.encoder_pulses = enc
            rel = enc - self._home_offset_pulses
            self.status.position_mm = (
                units.ballscrew_pulses_to_mm(rel, **self._conv_kwargs())
                + self.params.home_position_mm
            )
            self.status.motor_rpm = float(speed)
            self.status.axis_speed_mm_s = units.ballscrew_motor_rpm_to_axis_speed(
                self.status.motor_rpm,
                pitch_mm=self.params.pitch_mm,
                gear_ratio=self.params.gear_ratio,
            )
            self.status.last_error = ""
        except Exception as exc:  # noqa: BLE001
            self.status.last_error = str(exc)
            logger.debug("Ballscrew poll failed: %s", exc)
        return self.status
