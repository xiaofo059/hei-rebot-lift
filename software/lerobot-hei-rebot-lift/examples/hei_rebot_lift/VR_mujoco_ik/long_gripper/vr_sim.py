#!/usr/bin/env python
"""Run the stable HEI VR simulation with the independent long-gripper URDF."""

import sys

from long_gripper_config import BASE_MUJOCO_IK_DIR, LONG_GRIPPER_URDF


if str(BASE_MUJOCO_IK_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_MUJOCO_IK_DIR))

import hei_robot_vr_mujoco_sim as stable_sim  # noqa: E402


def main() -> None:
    stable_sim.DEFAULT_URDF = LONG_GRIPPER_URDF
    stable_sim.main()


if __name__ == "__main__":
    main()
