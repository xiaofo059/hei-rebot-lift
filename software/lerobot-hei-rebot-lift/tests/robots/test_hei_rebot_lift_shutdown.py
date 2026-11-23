"""Shutdown regression tests with simulated feedback; never open robot hardware."""

import io
import signal
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from lerobot.robots.hei_rebot_lift import safe_shutdown as shutdown
from lerobot.robots.hei_rebot_lift.config_hei_rebot_lift import HeiRebotLiftHostConfig
from lerobot.robots.hei_rebot_lift import hei_rebot_lift_host as host_module
from lerobot.motors.damiao_u2can.dm_can import Motor


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeMotor:
    def __init__(self, position):
        self.position = position
        self.velocity = 0.0
        self.status = 1
        self.feedback_sequence = 0

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getError(self):
        return self.status


class FakeControl:
    def __init__(self, clock):
        self.clock = clock
        self.commands = []
        self.fresh = True
        self.track = True

    def recv(self):
        pass

    def refresh_motor_status(self, motor):
        if self.fresh:
            motor.feedback_sequence += 1

    def control_Pos_Vel(self, motor, position, velocity):
        self.commands.append((self.clock.now, motor, position, velocity))
        if self.track:
            motor.position = position

    def control_pos_force(self, motor, position, velocity, current):
        self.commands.append((self.clock.now, motor, position, velocity))
        motor.position = position


def make_arm(name, clock):
    return SimpleNamespace(
        name=name, is_connected=True,
        motors=[FakeMotor(x) for x in [0.4, -0.5, -0.2, 0.1, 0.2, -0.1, -0.8]],
        motor_control=FakeControl(clock), arm_velocity_limits=np.full(6, 0.15),
        min_rad=np.full(7, -3.14), max_rad=np.full(7, 3.14),
        config=SimpleNamespace(gripper_force_velocity=1000, gripper_current=500),
    )


class TestShutdown(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.cfg = HeiRebotLiftHostConfig()
        self.timer = patch.object(shutdown, 'time', self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)
        self.arms = [make_arm(name, self.clock) for name in ('right', 'left')]
        self.robot = SimpleNamespace(
            right_arm=self.arms[0], left_arm=self.arms[1],
            chassis=SimpleNamespace(stop=Mock()), lift=SimpleNamespace(stop=Mock()),
            disconnect=Mock(),
        )

    def test_both_arms_return_smoothly_before_disconnect(self):
        def disconnected():
            for arm in self.arms:
                np.testing.assert_allclose([m.position for m in arm.motors[:6]], 0)
                self.assertEqual(arm.motors[6].position, -0.8)
            self.assertGreaterEqual(self.clock.now, 1.875 * 0.5 / 0.15 + 1.0)
        self.robot.disconnect.side_effect = disconnected
        shutdown.shutdown_robot(self.robot, self.cfg)
        self.robot.chassis.stop.assert_called_once()
        self.robot.lift.stop.assert_called_once()
        self.robot.disconnect.assert_called_once()
        for arm in self.arms:
            for motor in arm.motors[:6]:
                commands = [c for c in arm.motor_control.commands if c[1] is motor]
                self.assertTrue(all(c[3] <= 0.15 for c in commands))
                for a, b in zip(commands, commands[1:]):
                    self.assertLessEqual(abs(b[2]), abs(a[2]) + 1e-12)
                    if b[0] > a[0]:
                        self.assertLessEqual(abs(b[2] - a[2]) / (b[0] - a[0]), 0.150001)
            self.assertTrue(all(c[2] == -0.8 for c in arm.motor_control.commands if c[1] is arm.motors[6]))

    def test_cached_zero_feedback_is_not_arrival(self):
        arm = self.arms[0]
        for motor in arm.motors:
            motor.position = 0.0
        arm.motor_control.fresh = False
        with self.assertRaisesRegex(RuntimeError, 'fresh motor feedback'):
            shutdown.park_arms([arm], self.cfg, {})
        self.assertFalse(arm.motor_control.commands)

    def test_motor_fault_and_nonfinite_feedback_rejected(self):
        for status, position, message in [(13, 0.0, 'not enabled'), (1, float('nan'), 'non-finite')]:
            with self.subTest(status=status, position=position):
                arm = make_arm('fault', self.clock)
                arm.motors[0].status = status
                arm.motors[0].position = position
                with self.assertRaisesRegex(RuntimeError, message):
                    shutdown.park_arms([arm], self.cfg, {})
                self.assertFalse(arm.motor_control.commands)

    def test_stalled_joint_times_out(self):
        self.arms[0].motor_control.track = False
        self.cfg.shutdown_timeout_s = 9.0
        with self.assertRaisesRegex(RuntimeError, 'before timeout'):
            shutdown.park_arms(self.arms, self.cfg, {})

    def test_moving_joint_cannot_be_reported_as_parked(self):
        self.arms[0].motors[0].velocity = 0.1
        self.cfg.shutdown_timeout_s = 9.0
        with self.assertRaisesRegex(RuntimeError, 'before timeout'):
            shutdown.park_arms(self.arms, self.cfg, {})

    def test_zero_outside_limit_rejected(self):
        self.arms[0].min_rad[0] = 0.1
        with self.assertRaisesRegex(RuntimeError, 'zero outside'):
            shutdown.park_arms(self.arms, self.cfg, {})

    def test_fault_requires_explicit_supported_release(self):
        self.arms[0].motors[0].status = 13
        stream = io.StringIO('no\nDISABLE\n')
        reads = []
        def ready(*args):
            self.robot.disconnect.assert_not_called()
            reads.append(True)
            return [stream], [], []
        with patch.object(shutdown.sys, 'stdin', stream), patch.object(shutdown.select, 'select', ready):
            shutdown.shutdown_robot(self.robot, self.cfg)
        self.assertEqual(len(reads), 2)
        self.robot.disconnect.assert_called_once()

    def test_fault_with_closed_stdin_never_automatically_disables(self):
        class EndTest(BaseException):
            pass
        self.arms[0].motors[0].status = 13
        stream = io.StringIO('')
        def stop_test(seconds):
            if self.clock.now > 1.0:
                raise EndTest
            self.clock.now += seconds
        with patch.object(shutdown.sys, 'stdin', stream), patch.object(shutdown.select, 'select', return_value=([stream], [], [])), patch.object(self.clock, 'sleep', stop_test):
            with self.assertRaises(EndTest):
                shutdown.shutdown_robot(self.robot, self.cfg)
        self.robot.disconnect.assert_not_called()

    def test_shutdown_parameters_validated(self):
        for name, value in [('shutdown_velocity_rad_s', 1.0), ('shutdown_timeout_s', 0), ('shutdown_tolerance_rad', float('nan'))]:
            with self.subTest(name=name):
                cfg = HeiRebotLiftHostConfig()
                setattr(cfg, name, value)
                with self.assertRaises(ValueError):
                    shutdown.validate_shutdown_config(cfg)

    def test_repeated_ctrl_c_and_sigterm_do_not_interrupt_parking(self):
        previous = signal.getsignal(signal.SIGINT)
        with shutdown.shutdown_signals():
            handler = signal.getsignal(signal.SIGINT)
            with self.assertRaises(KeyboardInterrupt):
                handler(signal.SIGINT, None)
            handler(signal.SIGINT, None)
            handler(signal.SIGTERM, None)
        self.assertIs(signal.getsignal(signal.SIGINT), previous)
        with shutdown.shutdown_signals() as state:
            state['parking'] = True
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)

    def test_partial_connect_failure_also_uses_guarded_shutdown(self):
        cfg = host_module.HeiRebotLiftServerConfig()
        robot = Mock()
        robot.connect.side_effect = RuntimeError('partial connect')
        state = {'parking': False}
        with patch.object(host_module, 'HeiRebotLift', return_value=robot), patch.object(host_module, 'shutdown_robot') as park:
            with self.assertRaisesRegex(RuntimeError, 'partial connect'):
                host_module._run_host(cfg, state)
        self.assertTrue(state['parking'])
        park.assert_called_once_with(robot, cfg.host)
        robot.disconnect.assert_not_called()

    def test_status_sequence_changes_only_with_decoded_feedback(self):
        # Exercise the real status update method without constructing serial hardware.
        motor = Motor.__new__(Motor)
        motor.feedback_sequence = 0
        motor.recv_data(0.2, 0.0, 0.0, 1)
        self.assertEqual(motor.feedback_sequence, 1)
        self.assertEqual(motor.getPosition(), 0.2)
        motor.recv_data(0.3, 0.0, 0.0, 1)
        self.assertEqual(motor.feedback_sequence, 2)


if __name__ == '__main__':
    unittest.main()
