#!/usr/bin/python3
"""Terminal keyboard teleop for the chassis; communicates only through Twist."""

import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
import yaml


HELP = """
HEI 底盘键盘遥控（只在本终端读取按键）
  W / S : 前进 / 后退     A / D : 左移 / 右移
  Q / E : 左转 / 右转     1 / 2 / 3 : 低 / 中 / 高速
  空格 / K : 停车        X / Ctrl+C : 停车并退出
按住方向键时依靠系统按键重复；停止收到按键后自动停车。
切换档位、未知按键或终端失去前台控制时停车，需重新按方向键。
请保持该终端获得键盘焦点；建图、雷达和底盘控制器在其他终端运行。
"""


class KeyCommand:
    """Steady-clock key lease. No terminal, ROS or motor IO in this class."""

    DIRECTIONS = {
        "w": (1.0, 0.0, 0.0), "s": (-1.0, 0.0, 0.0),
        "a": (0.0, 1.0, 0.0), "d": (0.0, -1.0, 0.0),
        "q": (0.0, 0.0, 1.0), "e": (0.0, 0.0, -1.0),
    }

    def __init__(self, linear_speeds, angular_speeds, initial_gear, key_timeout):
        for values in (linear_speeds, angular_speeds):
            if len(values) != 3 or any(not math.isfinite(v) or v <= 0 for v in values):
                raise ValueError("linear_speeds/angular_speeds 需各含三个有限正数")
            if list(values) != sorted(values):
                raise ValueError("三个档位的速度需从小到大排列")
        if max(linear_speeds) > 0.25 or max(angular_speeds) > 0.5:
            raise ValueError("键盘速度上限为 0.25m/s、0.5rad/s，与默认底盘限幅一致")
        if initial_gear not in (1, 2, 3):
            raise ValueError("initial_gear 必须为 1、2 或 3")
        if not math.isfinite(key_timeout) or not 0.1 <= key_timeout <= 0.5:
            raise ValueError("key_timeout 需在 0.1～0.5 秒之间")
        self.linear_speeds = linear_speeds
        self.angular_speeds = angular_speeds
        self.gear = initial_gear
        self.timeout = key_timeout
        self.direction = (0.0, 0.0, 0.0)
        self.last_key = float("-inf")
        self.escape = ""

    def stop(self):
        self.direction = (0.0, 0.0, 0.0)
        self.last_key = float("-inf")

    def feed(self, data, now):
        """Return False for exit; discard CSI/SS3 sequences, e.g. arrow keys."""
        for char in data:
            # Do not interpret an arrow sequence's trailing A/D as a motion key.
            if self.escape:
                if self.escape == "esc":
                    self.escape = "sequence" if char in "[O" else ""
                elif "@" <= char <= "~":
                    self.escape = ""
                continue
            if char == "\x1b":
                self.escape = "esc"
                self.stop()
                continue
            key = char.lower()
            if key in ("x", "\x03", "\x04"):
                self.stop()
                return False
            if key in "123":
                self.gear = int(key)
                self.stop()
            elif key in self.DIRECTIONS:
                self.direction = self.DIRECTIONS[key]
                self.last_key = now
            else:
                self.stop()
        return True

    def velocity(self, now):
        if not 0 <= now - self.last_key < self.timeout:
            self.stop()
        linear = self.linear_speeds[self.gear - 1]
        angular = self.angular_speeds[self.gear - 1]
        return (self.direction[0] * linear, self.direction[1] * linear,
                self.direction[2] * angular)


class ChassisKeyboard(Node):
    def __init__(self):
        super().__init__("chassis_keyboard")
        config = Path(get_package_share_directory("hei_chassis_controller")) / "config" / "keyboard.yaml"
        with config.open(encoding="utf-8") as stream:
            defaults = yaml.safe_load(stream)["chassis_keyboard"]["ros__parameters"]
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.keys = KeyCommand(value("linear_speeds"), value("angular_speeds"),
                               value("initial_gear"), value("key_timeout"))
        rate = value("publish_rate")
        if not math.isfinite(rate) or not 10 <= rate <= 50:
            raise ValueError("publish_rate 需在 10～50Hz 之间")
        self.period = 1.0 / rate
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Twist, value("cmd_vel_topic"), qos)

    def publish_command(self, stop=False):
        msg = Twist()
        if not stop:
            msg.linear.x, msg.linear.y, msg.angular.z = self.keys.velocity(time.monotonic())
        self.publisher.publish(msg)

    def show_gear(self):
        g = self.keys.gear
        print(f"档位 {g}：平移 {self.keys.linear_speeds[g-1]:.2f} m/s，"
              f"旋转 {self.keys.angular_speeds[g-1]:.2f} rad/s；"
              f"按键超时 {self.keys.timeout:.2f}s", flush=True)


def main(args=None):
    if not sys.stdin.isatty():
        print("需要交互式终端：请使用 ros2 run hei_chassis_controller chassis_keyboard；"
              "SSH 时使用 ssh -t。", file=sys.stderr)
        return 2
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    # ros2 run's wrapper can exit before its child on SIGTERM. If it owns the
    # terminal session, that also sends SIGHUP to this process during cleanup.
    old_handlers = {s: signal.signal(s, request_stop)
                    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    node = None
    initialized = False
    try:
        # Keep the ROS context alive long enough to send zero on Ctrl+C/SIGTERM.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = ChassisKeyboard()
        print(HELP, flush=True)
        node.show_gear()
        tty.setcbreak(fd)
        termios.tcflush(fd, termios.TCIFLUSH)
        next_publish = time.monotonic()
        while not stopping and rclpy.ok():
            if os.tcgetpgrp(fd) != os.getpgrp():
                node.keys.stop()
                node.publish_command(stop=True)
                print("终端已不在前台，停车并退出。", flush=True)
                break
            readable, _, _ = select.select([fd], [], [], max(0.0, min(0.02, next_publish - time.monotonic())))
            if stopping:
                break
            if readable:
                data = os.read(fd, 64)
                if not data:
                    break
                gear = node.keys.gear
                if not node.keys.feed(data.decode("ascii", errors="replace"), time.monotonic()):
                    break
                if gear != node.keys.gear:
                    node.show_gear()
                # Send explicit stop/direction changes without waiting for the next tick.
                node.publish_command()
            now = time.monotonic()
            if now >= next_publish:
                node.publish_command()
                next_publish = now + node.period
            rclpy.spin_once(node, timeout_sec=0)
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"键盘遥控退出：{exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if node is not None and rclpy.ok():
                node.keys.stop()
                for _ in range(3):
                    node.publish_command(stop=True)
                    time.sleep(0.02)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, settings)
            except termios.error:
                pass  # Disconnected SSH terminal; controller timeout still applies.
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
