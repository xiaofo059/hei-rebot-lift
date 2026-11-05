"""Key lease tests; never opens a terminal or publishes to a real controller."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "chassis_keyboard", Path(__file__).parents[1] / "scripts" / "chassis_keyboard.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
KeyCommand = module.KeyCommand


def keys(**overrides):
    values = dict(linear_speeds=[0.05, 0.10, 0.15], angular_speeds=[0.15, 0.30, 0.45],
                  initial_gear=1, key_timeout=0.25)
    values.update(overrides)
    return KeyCommand(**values)


@pytest.mark.parametrize("key,expected", [
    ("W", (0.05, 0., 0.)), ("s", (-0.05, 0., 0.)),
    ("a", (0., 0.05, 0.)), ("D", (0., -0.05, 0.)),
    ("q", (0., 0., 0.15)), ("E", (0., 0., -0.15)),
])
def test_ros_axes(key, expected):
    command = keys()
    assert command.velocity(9.0) == (0., 0., 0.)
    assert command.feed(key, 10.0)
    assert command.velocity(10.01) == expected


def test_key_release_stops_and_old_direction_cannot_resume():
    command = keys()
    command.feed("w", 10.)
    assert command.velocity(10.24)[0] == 0.05
    assert command.velocity(10.25) == (0., 0., 0.)
    assert command.velocity(9.9) == (0., 0., 0.)
    command.feed("w", 11.)
    assert command.velocity(11.01)[0] == 0.05


def test_gear_change_stops_until_new_direction():
    command = keys()
    command.feed("w", 10.)
    command.feed("3", 10.1)
    assert command.velocity(10.11) == (0., 0., 0.)
    command.feed("w", 10.12)
    assert command.velocity(10.13) == (0.15, 0., 0.)


@pytest.mark.parametrize("stop_key", [" ", "k", "?", "\n", "\x1b[A", "\x1bOD"])
def test_stop_and_unhandled_keys_cancel_motion(stop_key):
    command = keys()
    command.feed("w", 10.)
    command.feed(stop_key, 10.01)
    assert command.velocity(10.02) == (0., 0., 0.)


def test_fragmented_arrow_does_not_turn_into_left_command():
    command = keys()
    command.feed("w", 10.)
    for char in "\x1b[A":
        command.feed(char, 10.1)
        assert command.velocity(10.11) == (0., 0., 0.)
    command.feed("a", 10.12)
    assert command.velocity(10.13) == (0., 0.05, 0.)


@pytest.mark.parametrize("exit_key", ["x", "X", "\x03", "\x04"])
def test_exit_cancels_motion(exit_key):
    command = keys()
    command.feed("w", 10.)
    assert not command.feed(exit_key, 10.1)
    assert command.velocity(10.11) == (0., 0., 0.)


@pytest.mark.parametrize("overrides", [
    {"linear_speeds": [float("nan"), 0.1, 0.15]},
    {"linear_speeds": [0.1, 0.05, 0.15]},
    {"angular_speeds": [0.1, 0.2, 0.6]},
    {"linear_speeds": [0.05, 0.1, 0.3]},
    {"key_timeout": 0.0}, {"key_timeout": float("inf")},
    {"key_timeout": 1.0}, {"initial_gear": 4},
])
def test_rejects_unsafe_or_invalid_settings(overrides):
    with pytest.raises(ValueError):
        keys(**overrides)
