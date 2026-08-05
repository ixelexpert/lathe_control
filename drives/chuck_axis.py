"""Chuck axis — A6-RS speed mode over Modbus."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .modbus_bus import ModbusBus
from . import units

logger = logging.getLogger(__name__)

# Registers
C00_00 = 0x0000  # control mode
C03_20 = 0x0320  # speed reference source
C04_11 = 0x0411  # virtual enable
C12_00 = 0x1200  # speed profile mode
C12_06 = 0x1206  # speed (rpm, I16)
C12_0A = 0x120A  # accel ms (U32)
C12_0C = 0x120C  # decel ms (U32)
U40_01 = 0x4001  # actual motor speed (I32 typically 2 regs)


@dataclass
class ChuckParams:
    slave_id: int = 2
    baud: int = 115200
    gear_ratio: float = 4.0
    axis_speed_rpm: float = 60.0
    acceleration_ms: float = 200.0
    deceleration_ms: float = 200.0
    duration_s: float = 0.0
    start_delay_s: float = 0.0
    end_delay_s: float = 0.0
    invert_direction: bool = False
    max_chuck_rpm: float = 300.0


@dataclass
class ChuckStatus:
    enabled: bool = False
    spinning: bool = False
    motor_rpm: float = 0.0
    axis_rpm: float = 0.0
    fault: str = ""
    last_error: str = ""


class ChuckAxis:
    def __init__(self, bus: ModbusBus, params: ChuckParams | None = None) -> None:
        self.bus = bus
        self.params = params or ChuckParams()
        self.status = ChuckStatus()
        self._configured = False

    @property
    def motor_speed_rpm(self) -> float:
        return units.chuck_axis_to_motor_rpm(
            self.params.axis_speed_rpm, self.params.gear_ratio
        )

    def set_axis_speed(self, chuck_rpm: float) -> None:
        self.params.axis_speed_rpm = units.clamp(
            chuck_rpm, -self.params.max_chuck_rpm, self.params.max_chuck_rpm
        )

    def set_motor_speed(self, motor_rpm: float) -> None:
        chuck = units.chuck_motor_to_axis_rpm(motor_rpm, self.params.gear_ratio)
        self.set_axis_speed(chuck)

    def configure(self) -> None:
        sid = self.params.slave_id
        self.bus.write_u16(sid, C00_00, 1)  # speed mode
        self.bus.write_u16(sid, C03_20, 3)  # internal planned speed
        self.bus.write_u16(sid, C12_00, 1)  # cyclic
        self.apply_motion_params()
        self._configured = True
        logger.info("Chuck axis configured (slave %s)", sid)

    def apply_motion_params(self) -> None:
        sid = self.params.slave_id
        rpm = self._command_motor_rpm()
        self.bus.write_i16(sid, C12_06, int(round(rpm)))
        self.bus.write_u32(sid, C12_0A, int(round(self.params.acceleration_ms)))
        self.bus.write_u32(sid, C12_0C, int(round(self.params.deceleration_ms)))

    def _command_motor_rpm(self) -> float:
        rpm = self.motor_speed_rpm
        if self.params.invert_direction:
            rpm = -rpm
        # CCW positive convention: positive axis_speed_rpm -> positive motor command
        return rpm

    def enable(self) -> None:
        if not self._configured:
            self.configure()
        else:
            self.apply_motion_params()
        self.bus.write_u16(self.params.slave_id, C04_11, 1)
        self.status.enabled = True
        self.status.spinning = True

    def disable(self) -> None:
        try:
            self.bus.write_u16(self.params.slave_id, C04_11, 0)
        except Exception as exc:  # noqa: BLE001 — best-effort stop
            self.status.last_error = str(exc)
            logger.exception("Chuck disable failed")
        self.status.enabled = False
        self.status.spinning = False

    def start(self) -> None:
        """Enable and spin at configured speed."""
        self.enable()

    def stop(self) -> None:
        self.disable()

    def poll(self) -> ChuckStatus:
        try:
            raw = self.bus.read_i32(self.params.slave_id, U40_01)
            # Drive reports motor rpm; scale may be 1 rpm units
            self.status.motor_rpm = float(raw)
            self.status.axis_rpm = units.chuck_motor_to_axis_rpm(
                self.status.motor_rpm, self.params.gear_ratio
            )
            self.status.last_error = ""
        except Exception as exc:  # noqa: BLE001
            self.status.last_error = str(exc)
            logger.debug("Chuck poll failed: %s", exc)
        return self.status

    def run_for_duration(self, duration_s: float, *, stop_event=None) -> None:
        """Blocking helper used by cycle engine."""
        self.start()
        end = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < end:
            if stop_event is not None and stop_event.is_set():
                break
            time.sleep(0.05)
        self.stop()
