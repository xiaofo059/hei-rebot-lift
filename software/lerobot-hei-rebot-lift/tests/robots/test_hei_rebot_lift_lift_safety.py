"""Fail-safe behavior of the HEI lift when limit-switch feedback changes."""

import unittest
from types import SimpleNamespace

from lerobot.robots.hei_rebot_lift.config_hei_rebot_lift import HeiRebotLiftConfig
from lerobot.robots.hei_rebot_lift.hei_rebot_lift import _LiftRuntime


class TestLiftSafety(unittest.TestCase):
    def setUp(self):
        config = HeiRebotLiftConfig(cameras={})
        self.lift = _LiftRuntime("unused-motor", "unused-io", config)
        self.commands = []
        self.lift.motor = object()
        self.lift.motor_control = SimpleNamespace(
            control_Vel=lambda motor, speed: self.commands.append(speed)
        )
        self.lift.reader = SimpleNamespace(
            online=True, upper_active=False, lower_active=False, both_active=False
        )
        self.lift._poll_height = lambda: -10.0
        self.lift.height_mm = -10.0
        self.lift.last_velocity_rad_s = -5.0
        self.lift.last_command_time_s = 0.0

    def test_stale_io_stops_immediately(self):
        self.lift.reader.online = False
        with self.assertRaisesRegex(RuntimeError, "stale or offline"):
            self.lift.command_position(0.0)
        self.assertEqual(self.commands, [0.0])
        self.assertEqual(self.lift.last_velocity_rad_s, 0.0)

    def test_upper_limit_stops_without_acceleration_ramp(self):
        self.lift.reader.upper_active = True
        self.lift.command_position(0.0)
        self.assertEqual(self.commands, [0.0])

    def test_lower_limit_stops_without_acceleration_ramp(self):
        self.lift.reader.lower_active = True
        self.lift.command_position(-20.0)
        self.assertEqual(self.commands, [0.0])

    def test_conflicting_limits_stop_and_raise(self):
        self.lift.reader.both_active = True
        with self.assertRaisesRegex(RuntimeError, "Both lift limit switches"):
            self.lift.command_position(0.0)
        self.assertEqual(self.commands, [0.0])


if __name__ == "__main__":
    unittest.main()
