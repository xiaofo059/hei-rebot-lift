# AGX upstream source patches

This directory records historical patches from before the ROS package sources were
tracked directly by dev_agx. Do not reapply this patch to the current vendored source.
Current gravity alignment is controlled by the FAST-LIO2 YAML `gravity_align`
parameter; the AGX profiles enable it explicitly. Initialization sample count is
controlled by `imu_init_num`. The validation below describes the earlier revision.

## FAST-LIO2 gravity-aligned initialization

Base upstream commit: `a4743b095409588842a5b30ddfa27e29d2f99164`.
`fast-lio2-gravity-alignment.patch` is already applied to the AGX FAST-LIO2 checkout.
The patch records changes to the independently cloned source excluded from the parent repository.
For a fresh checkout at the base commit, run from `extensions/ros2_ws/src/fast_lio`:

```bash
git apply ../../../patches/fast-lio2-gravity-alignment.patch
```

Initialization rotates the measured stationary specific-force direction onto world +Z
using the shortest rotation and sets world gravity along -Z. Absolute heading remains
unobservable from gravity alone. The original gravity magnitude convention is retained.
Finite nonzero acceleration is required; invalid input resets initialization.
This changes the initial attitude, not just a displayed TF. Registered clouds, odometry
and deskewing share the same initialized filter state; LiDAR-IMU extrinsics are unchanged.
No additional TF broadcaster is introduced.

Keep the sensor stationary during startup. Motion acceleration and sensor bias can
still corrupt gravity estimation. The frame is initialized level; gravity remains
an estimated filter state, not a hard constraint reapplied on every frame.
The origin stays at startup position, not at ground height. Existing RTAB-Map databases
and PCD files are not rotated by this patch; old maps require their own alignment check.
The current RTAB-Map configuration also does not constrain map roll/pitch to gravity.

Build and initialization regression (from `extensions/ros2_ws` after sourcing ROS):

```bash
colcon build --packages-select fast_lio --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/fast_lio --output-on-failure
```

The test invokes the actual IMU initialization path with horizontal, tilted, sideways,
inverted, varying-scale and invalid input, and verifies attitude/gravity consistency,
unchanged origin and recovery. No hardware nodes are started by the test.
Existing processes keep the old program until restarted.

## Recorded validation

AGX native build/install passed. The actual IMU Process initialization test passed
127 cases (CTest: gravity_alignment_initialization).
An isolated ROS domain 197 / localhost-only test published synthetic Livox scans
and IMU for a stationary sensor tilted roll=20 degrees, pitch=-15 degrees, using
known fixed extrinsics. Both the original and patched binaries were tested.
Original first floor cloud Z standard deviation: 0.9898 m; patched: 0.00128 m.
The patched first-frame world specific-force unit vector was approximately
[-0.000493, -0.000221, 0.99999985], confirming initialization/output alignment.

This is INITIAL alignment validation, not proof of continuous attitude stability.
In the 8-second synthetic sequence, the final floor cloud had Z standard deviation
0.03486 m and median height -0.946 m versus nominal -1 m; later attitude/height
variation remains unaccepted. An earlier test asserting stricter whole-sequence
stability failed, both with online extrinsics and with fixed known extrinsics.
No such failure was hidden by changing production thresholds. Real stationary
and moving hardware validation remains pending.

Evidence: extensions/ros2_ws/log/gravity_alignment_20261028/. All software test
processes exited and original runtime log paths/inodes were restored. Existing
user nodes were not restarted by this work.
