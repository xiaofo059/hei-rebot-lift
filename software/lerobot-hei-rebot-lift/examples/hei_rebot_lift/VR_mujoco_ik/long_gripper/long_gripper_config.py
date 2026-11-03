#!/usr/bin/env python
"""Shared paths and geometry constants for the independent long-gripper variant."""

from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
BASE_MUJOCO_IK_DIR = BASE_DIR.parent / "mujoco_ik"
LONG_GRIPPER_MODEL_DIR = BASE_DIR / "model"
LONG_GRIPPER_URDF = LONG_GRIPPER_MODEL_DIR / "urdf" / "HEI_robot_urdf_long_gripper.urdf"

# Both values are measured in each finger mesh's local +Z direction.  The URDF
# finger joint rotation maps that axis to the arm's forward direction.
ORIGINAL_FINGER_TIP_M = 0.0709093213
LONG_GRIPPER_FINGER_TIP_M = 0.10510
LONG_GRIPPER_EXTENSION_M = LONG_GRIPPER_FINGER_TIP_M - ORIGINAL_FINGER_TIP_M

# The original invisible physics pad spans local Z [-0.023, 0.047] m.  Extend
# only its distal end by the same amount as the new finger while preserving the
# original proximal edge and the clearance between pad end and visual tip.
LONG_GRIPPER_PAD_LOCAL_Z_M = 0.02909533935
LONG_GRIPPER_PAD_HALF_SIZE_M = (0.005, 0.035, 0.05209533935)
