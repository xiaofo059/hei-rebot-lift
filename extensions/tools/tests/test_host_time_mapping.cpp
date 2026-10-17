// HEI-LIDAR-20261017-02-FIX: synthetic regression test for src/comm/host_time.h.
//
// Build (after the driver patch is applied):
//   g++ -std=c++11 -O2 -Wall -Wextra -I <driver>/src/comm/ test_host_time_mapping.cpp
//
// The test never changes the host clock; it feeds synthetic steady-time samples
// into the pure mapping helper and checks the first-use invariant that Codex
// review asked for:
//   * the VERY FIRST time helper call in main() is HostNowNs(), so an
//     anchor/steady ordering bug on first initialisation is observable;
//   * the first live value is not earlier than the captured epoch anchor
//     (uint64 underflow guard);
//   * anchors are captured exactly once and the mapping stays monotonic.
#include <chrono>
#include <cstdint>
#include <cstdio>

#include "host_time.h"

namespace {
int g_failures = 0;

void Check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  } else {
    std::printf("ok  : %s\n", what);
  }
}
}  // namespace

int main() {
  // Codex review: this must be the FIRST time helper call in the process.
  // If HostNowNs() sampled steady_clock before initialising the anchors, the
  // first value could underflow below the epoch anchor and this check would
  // catch it.
  const std::uint64_t first = livox_ros::HostNowNs();

  // Now (and only now) read the anchors: they must already be initialised and
  // stable.
  const livox_ros::HostClockAnchors& a1 = livox_ros::HostClockAnchorsInstance();
  const livox_ros::HostClockAnchors& a2 = livox_ros::HostClockAnchorsInstance();
  Check(&a1 == &a2, "anchors are captured exactly once");
  Check(first >= a1.epoch_ns, "first HostNowNs() is not earlier than the epoch anchor");
  Check(a1.epoch_ns > 1000000000000000000ULL, "epoch anchor looks like a real epoch");

  const std::uint64_t second = livox_ros::HostNowNs();
  Check(second >= first, "HostNowNs() is non-decreasing");

  // Pure mapping checks (no clocks involved): monotonic under advancing steady
  // time and independent of any hypothetical CLOCK_REALTIME step.
  const std::uint64_t anchor_epoch = 1792146782000000000ULL;  // ~2026-10-16
  const std::uint64_t anchor_steady = 5000000000ULL;          // 5 s after boot
  Check(livox_ros::MapSteadyToEpochNs(anchor_epoch, anchor_steady, anchor_steady) ==
            anchor_epoch,
        "mapping at anchor returns anchor epoch");
  std::uint64_t prev = 0;
  bool mono = true;
  for (std::uint64_t d = 0; d <= 2000000000ULL; d += 100000000ULL) {
    const std::uint64_t mapped =
        livox_ros::MapSteadyToEpochNs(anchor_epoch, anchor_steady, anchor_steady + d);
    if (d >= 100000000ULL && mapped <= prev) mono = false;
    prev = mapped;
  }
  Check(mono, "mapping is strictly monotonic for advancing steady time");
  const std::uint64_t before_step =
      livox_ros::MapSteadyToEpochNs(anchor_epoch, anchor_steady, anchor_steady + 1000000000ULL);
  const std::uint64_t after_step =
      livox_ros::MapSteadyToEpochNs(anchor_epoch, anchor_steady, anchor_steady + 1100000000ULL);
  Check(after_step > before_step,
        "mapping unchanged by a hypothetical wall-clock step");

  // Anchor fidelity: the live mapping still tracks CLOCK_REALTIME within 2 s
  // (generous; the test itself runs in well under a second).
  const auto sys_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  const std::int64_t skew =
      static_cast<std::int64_t>(first) - static_cast<std::int64_t>(sys_ns);
  Check(skew < 2000000000LL && skew > -2000000000LL,
        "HostNowNs() stays within 2 s of CLOCK_REALTIME");

  if (g_failures == 0) {
    std::printf("\nALL CHECKS PASSED\n");
    return 0;
  }
  std::printf("\n%d CHECK(S) FAILED\n", g_failures);
  return 1;
}
