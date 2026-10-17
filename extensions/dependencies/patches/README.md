# Third-party patches for the MID-360S underlay

These are **tracked patches** against the frozen third-party revisions pinned in
`extensions/dependencies/livox.repos`. They are applied by
`extensions/tools/build_mid360s_underlay.sh`; the frozen baseline SHAs stay
recorded and verifiable, and a worktree that contains anything *other* than
"frozen SHA + these patches" is rejected.

## Patch set

| File | Repo | Purpose |
| --- | --- | --- |
| `livox_sdk2/0001-export-only-public-c-api.patch` | Livox-SDK2 `f5d9375f` (v1.3.1) | Link the shared SDK library with a symbol version script so only the public C API from `include/livox_lidar_api.h` is exported. Hides the bundled spdlog 1.3.1 / fmt weak symbols and internal `livox::lidar::*` symbols. |
| `livox_ros_driver2/0001-monotonic-host-clock.patch` | livox_ros_driver2 `13eb05e4` (1.2.6) | For the confirmed NoSync path (`time_type == 0`), stamp point/IMU messages with a monotonic, epoch-anchored host clock (one `system_clock` anchor + `steady_clock` elapsed) and schedule the non-sync publish loop with `steady_clock`. PTP/GPS path is unchanged. |

## Why (evidence)

### 1. `libspdlog.so.1` vs bundled spdlog 1.3.1 (driver exit SIGBUS/SIGABRT)

The ROS 2 Humble logging backend (`librcl_logging_spdlog.so`) loads the system
`libspdlog.so.1` (1.9.2) into the driver process. The SDK bundled spdlog 1.3.1
is header-only and was exported with default visibility (421 `spdlog::*` + 124
`fmt::*` dynamic symbols in the frozen build), so its weak C++ template symbols
were preempted by 1.9.2. The SDK global
`std::shared_ptr<spdlog::logger>` was destroyed through
`std::_Sp_counted_ptr_inplace<spdlog::logger, ...>::_M_dispose()` **from
libspdlog.so.1** although the object was constructed with the 1.3.1 layout.

* coredump (formal run PID 344190, SIGBUS): frame #0 in `libspdlog.so.1`
  (`_M_dispose`), frame #1 in `liblivox_lidar_sdk_shared.so`
  (`~shared_ptr<spdlog::logger>`), caller `__run_exit_handlers`.
* No-hardware reproducer: `dlopen(system libspdlog, RTLD_GLOBAL)` then
  `dlopen(SDK)` + `LivoxLidarSdkInit(nullptr, nullptr, nullptr)` -> normal exit
  SIGBUS(-7); SDK alone -> exit 0. See
  `docs/agent/evidence/mid360s/20261017-fix/`.

### 2. Host CLOCK_REALTIME step (LiDAR source-time FAIL)

The MID-360S sends `time_type = 0` (NoSync) on both point-cloud (56301) and IMU
(56401) ports (passive UDP capture, 2026-10-17). Upstream substituted
`std::chrono::high_resolution_clock` (CLOCK_REALTIME) for those timestamps and
also used it for non-sync frame scheduling. On 2026-10-16 the host
CLOCK_REALTIME stepped back ~598 ms:

* true insertion (`messages.id`) order: IMU has exactly ONE negative header
  delta (-598.307 ms at id 73138). LiDAR has a REAL discontinuity pair: the
  aggregated frame id 73285 has its `timebase` -504.288 ms behind the previous
  true-order frame (73139), followed by a REAL +713.432 ms forward gap to frame
  73301. The original QC FAIL therefore stands: the source timeline genuinely
  breaks at the host clock step;
* the QC read the bag in rosbag2 timestamp-sorted order, which re-orders the
  step region: it reports 2 LiDAR regressions (-600.660 ms id 73012,
  -402.556 ms id 73285) and gaps +700.107 ms (id 73139) / +713.432 ms
  (id 73301). Only the id 73285 regression and the id 73301 gap are real in
  true order; the id 73012 regression and the id 73139 gap are sorting
  artefacts (pre-step frames whose receive stamps land after the step).
  Critically, sorting makes the IMU stream show 0 regressions, i.e. it HIDES
  the real -598.307 ms IMU clock step;
* the non-sync scheduler stalled for the whole step and then emitted a
  140,736-point frame spanning 702.866 ms (0.703 s x ~200 k points/s),
  `timebase` = first buffered point.

The patch makes the NoSync packet timeline monotonic; it does **not** hide
genuine device-time regressions on the PTP/GPS path and it is **not** hardware
time synchronisation.

## Applying / reverting

```bash
# build applies the patches automatically when the worktrees are at the frozen SHA
extensions/tools/build_mid360s_underlay.sh
# restore the third-party worktrees to the frozen SHA (no build)
extensions/tools/build_mid360s_underlay.sh --revert-patches
```

The build script refuses to build when a worktree contains unknown tracked
changes, or when the patched content does not byte-match "frozen SHA + patches".

## Notes

* All patch content is reversible; `git apply --reverse --check` must pass in a
  patched worktree (the build script enforces this).
* Patch files are generated from the frozen trees, not hand-edited; keep
  `livox_sdk2/public_c_api.txt` in sync with `livox_lidar_api.h` when bumping the SDK.
