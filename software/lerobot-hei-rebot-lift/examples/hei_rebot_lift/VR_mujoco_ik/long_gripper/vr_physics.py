#!/usr/bin/env python
"""Run contact-rich VR simulation with long-gripper-specific contact pads."""

import sys

from long_gripper_config import (
    BASE_MUJOCO_IK_DIR,
    LONG_GRIPPER_PAD_HALF_SIZE_M,
    LONG_GRIPPER_PAD_LOCAL_Z_M,
    LONG_GRIPPER_URDF,
)


if str(BASE_MUJOCO_IK_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_MUJOCO_IK_DIR))

import hei_robot_mujoco_physics_scene as physics_scene  # noqa: E402
import hei_robot_vr_mujoco_sim as stable_sim  # noqa: E402


def main() -> None:
    # The physics builder reads these module constants when it creates the
    # invisible rectangular pads.  Patching them here keeps the original
    # short-gripper physics program and constants unchanged.
    physics_scene.FINGER_PAD_HALF_SIZE_M = LONG_GRIPPER_PAD_HALF_SIZE_M
    physics_scene.FINGER_PAD_LOCAL_Z_M = LONG_GRIPPER_PAD_LOCAL_Z_M
    stable_sim.DEFAULT_URDF = LONG_GRIPPER_URDF

    # Import after configuring the shared builders.  The imported simulator
    # keeps using the same validated control implementation but receives the
    # long model and pad dimensions above.
    import hei_robot_vr_mujoco_physics as physics_controller  # noqa: PLC0415

    physics_controller.main()


if __name__ == "__main__":
    main()
