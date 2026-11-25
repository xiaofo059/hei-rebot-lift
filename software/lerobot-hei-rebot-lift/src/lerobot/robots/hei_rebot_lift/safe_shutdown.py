"""Host-only parking before releasing arm torque; never changes motor calibration."""

import logging
import math
import select
import signal
import sys
import time
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

import numpy as np

from .config_hei_rebot_lift import HeiRebotLiftHostConfig

logger = logging.getLogger(__name__)


def validate_shutdown_config(cfg: HeiRebotLiftHostConfig) -> None:
    for name in ("shutdown_velocity_rad_s", "shutdown_timeout_s", "shutdown_tolerance_rad",
                 "shutdown_settle_s"):
        value = getattr(cfg, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if cfg.shutdown_velocity_rad_s > 0.3:
        raise ValueError("shutdown_velocity_rad_s must not exceed 0.3 rad/s")


@contextmanager
def shutdown_signals() -> Iterator[dict[str, bool]]:
    """First SIGINT/SIGTERM requests parking; subsequent signals cannot skip it."""
    state = {"requested": False, "parking": False}

    def request_shutdown(signum: int, frame: Any) -> None:
        if not state["requested"] and not state["parking"]:
            state["requested"] = True
            raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, request_shutdown) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield state
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _feedback(arm: Any) -> tuple[np.ndarray, np.ndarray]:
    """Require a new status packet from every motor, not default/cached zeros."""
    if len(arm.motors) != 7:
        raise RuntimeError(f"{arm.name}: incomplete motor initialization")
    # Drain packets already waiting before requiring another feedback update.
    arm.motor_control.recv()
    sequence = [motor.feedback_sequence for motor in arm.motors]
    for motor in arm.motors:
        arm.motor_control.refresh_motor_status(motor)
    deadline = time.monotonic() + 0.3
    while any(m.feedback_sequence <= old for m, old in zip(arm.motors, sequence, strict=True)):
        if time.monotonic() >= deadline:
            raise RuntimeError(f"{arm.name}: timed out waiting for fresh motor feedback")
        time.sleep(0.002)
        arm.motor_control.recv()
    positions = np.array([m.getPosition() for m in arm.motors], dtype=float)
    velocities = np.array([m.getVelocity() for m in arm.motors], dtype=float)
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
        raise RuntimeError(f"{arm.name}: non-finite motor feedback")
    for index, motor in enumerate(arm.motors):
        # DM status nibble: 0 = disabled, 1 = enabled, 8..14 = faults.
        if motor.getError() != 1:
            raise RuntimeError(f"{arm.name} motor {index + 1}: not enabled, status={motor.getError():#x}")
    return positions, velocities


def _command_raw(arm: Any, target: np.ndarray, speed: float) -> None:
    # Mechanical zero is raw motor 0 rad, not the teleoperation/model offset.
    for index, motor in enumerate(arm.motors[:6]):
        arm.motor_control.control_Pos_Vel(
            motor, float(target[index]), min(speed, float(arm.arm_velocity_limits[index]))
        )
    arm.motor_control.control_pos_force(
        arm.motors[6], float(target[6]),
        float(arm.config.gripper_force_velocity), float(arm.config.gripper_current),
    )


def park_arms(arms: list[Any], cfg: HeiRebotLiftHostConfig, hold: dict[str, np.ndarray]) -> None:
    """Return both arms together on a slow quintic joint trajectory and verify rest."""
    validate_shutdown_config(cfg)
    starts: dict[str, np.ndarray] = {}
    for arm in arms:
        if (not np.all(np.isfinite(arm.arm_velocity_limits))
                or np.any(arm.arm_velocity_limits <= 0)
                or not np.all(np.isfinite(arm.min_rad[:6]))
                or not np.all(np.isfinite(arm.max_rad[:6]))
                or np.any(arm.min_rad[:6] > 0) or np.any(arm.max_rad[:6] < 0)):
            raise RuntimeError(f"{arm.name}: invalid limits or zero outside joint limits")
        positions, _ = _feedback(arm)
        if np.any(positions[:6] < arm.min_rad[:6] - cfg.shutdown_tolerance_rad) or np.any(
            positions[:6] > arm.max_rad[:6] + cfg.shutdown_tolerance_rad
        ):
            raise RuntimeError(f"{arm.name}: current pose outside joint limits")
        starts[arm.name] = positions.copy()
        hold[arm.name] = positions.copy()
        # Cancel the last teleoperation target before preparing the return.
        _command_raw(arm, positions, cfg.shutdown_velocity_rad_s)
    if not arms:
        return
    duration = max(3.0, *(float(np.max(
        1.875 * np.abs(starts[a.name][:6]) / np.minimum(a.arm_velocity_limits, cfg.shutdown_velocity_rad_s)
    )) for a in arms))
    if duration + cfg.shutdown_settle_s >= cfg.shutdown_timeout_s:
        raise RuntimeError("Parking trajectory exceeds shutdown_timeout_s")
    started = time.monotonic()
    settled_since = None
    while True:
        tick = time.monotonic()
        elapsed = tick - started
        if elapsed > cfg.shutdown_timeout_s:
            raise RuntimeError("Arms did not reach mechanical zero before timeout")
        u = min(elapsed / duration, 1.0)
        fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
        at_rest = elapsed >= duration
        for arm in arms:
            positions, velocities = _feedback(arm)
            hold[arm.name] = positions.copy()
            # Keep the gripper's initial measured position for the whole return.
            hold[arm.name][6] = starts[arm.name][6]
            target = starts[arm.name].copy()
            target[:6] *= 1.0 - fraction
            _command_raw(arm, target, cfg.shutdown_velocity_rad_s)
            at_rest = at_rest and bool(
                np.max(np.abs(positions[:6])) <= cfg.shutdown_tolerance_rad
                and np.max(np.abs(velocities[:6])) <= 0.03
            )
        now = time.monotonic()
        settled_since = (now if settled_since is None else settled_since) if at_rest else None
        if settled_since is not None and now - settled_since >= cfg.shutdown_settle_s:
            logger.warning("Both arms verified at mechanical zero; disconnecting.")
            return
        time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))


def shutdown_robot(robot: Any, cfg: HeiRebotLiftHostConfig) -> None:
    """Never fall through to torque disable after a failed parking attempt."""
    for runtime in (robot.chassis, robot.lift):
        try:
            runtime.stop()
        except Exception:
            logger.exception("Failed to stop chassis/lift during shutdown")
    arms = [arm for arm in (robot.right_arm, robot.left_arm) if arm.is_connected]
    hold: dict[str, np.ndarray] = {}
    try:
        logger.warning("Stopping teleoperation; returning arms to mechanical zero slowly.")
        park_arms(arms, cfg, hold)
    except Exception:
        logger.exception("Arm parking failed. Automatic torque disable is BLOCKED.")
        logger.error(
            "Service remains alive and attempts position hold. Support both arms physically. "
            "Only after support is secure, type DISABLE then Enter to release torque and exit. "
            "Repeated Ctrl+C does not release torque. Communication/power failure cannot guarantee hold."
        )
        # No automatic retry motion after a fault. Maintain last measured safe targets
        # where communication is still healthy, until an operator supports the arms.
        stdin_open = True
        while True:
            for arm in arms:
                try:
                    positions, _ = _feedback(arm)
                    if arm.name not in hold:
                        hold[arm.name] = positions.copy()
                    _command_raw(arm, hold[arm.name], cfg.shutdown_velocity_rad_s)
                except Exception:
                    pass  # The fault is already latched and logged; never disable here.
            if stdin_open:
                try:
                    ready, _, _ = select.select([sys.stdin], [], [], 0)
                    if ready:
                        line = sys.stdin.readline()
                        if not line:
                            stdin_open = False
                        elif line.strip() == "DISABLE":
                            break
                except (OSError, ValueError):
                    stdin_open = False
            time.sleep(0.05)
    robot.disconnect()
