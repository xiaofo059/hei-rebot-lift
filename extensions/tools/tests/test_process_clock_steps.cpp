// HEI-LIDAR-20261017-02-FIX-TEST: process-local CLOCK_REALTIME step injection.
//
// SYNTHETIC FAULT INJECTION — NOT A HARDWARE / OS EXPERIMENT
// ----------------------------------------------------------
// This test subclasses nothing and changes no system state: it interposes
// clock_gettime() inside THIS process only, forwarding to the real syscall and
// adding an offset to CLOCK_REALTIME results. The OS clock, NTP, the running
// driver and any concurrent acquisition process are untouched. There is no
// hardware access and no SDK/driver library is loaded or replaced.
//
// Purpose: keep the recovery of the 2026-10-16 host-clock step as a reproducible
// regression. The production mapping in host_time.h captures one system_clock
// epoch anchor and then advances on steady_clock elapsed time, so a
// CLOCK_REALTIME step must NOT move production timestamps:
//
//   original incident (formal bag 2026-10-16): CLOCK_REALTIME -598.3 ms
//   Codex independent run 2026-10-17 (this test):  CLOCK_REALTIME -589.931 ms /
//     HostNowNs +10.069 ms;  CLOCK_REALTIME +1310.063 ms / HostNowNs +10.063 ms
//
// Pass criteria:
//   1. injection really happened: measured wall-clock deltas show a backstep
//      <= -500 ms and a forward step >= +1200 ms;
//   2. production mapping is stable: HostNowNs() stays strictly increasing with
//      each 10 ms sleep and neither delta exceeds 100 ms;
//   3. anchors were taken from the real clock before injection (first call is
//      HostNowNs(), as in production), and no value goes backwards.
//
// Build: g++ -std=c++11 -O2 -o test_process_clock_steps test_process_clock_steps.cpp
//        -I <driver>/src/comm/ -pthread
// Note: the interposer is a main-executable definition of clock_gettime; it is
// effective for libstdc++ std::chrono::system_clock in this process. This is
// portable C/POSIX (syscall SYS_clock_gettime) and is expected to behave the
// same on aarch64, but it has only been run on this x86_64 laptop.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>

#include "host_time.h"

namespace {

std::atomic<long long> wall_offset_ns{0};

long long WallNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

// Process-local interposer: forward to the real syscall, then shift only the
// CLOCK_REALTIME result by the current synthetic offset.
extern "C" int clock_gettime(clockid_t id, struct timespec* ts) noexcept {
  const int rc = static_cast<int>(syscall(SYS_clock_gettime, id, ts));
  if (rc == 0 && id == CLOCK_REALTIME) {
    long long ns = static_cast<long long>(ts->tv_sec) * 1000000000LL +
                   static_cast<long long>(ts->tv_nsec) + wall_offset_ns.load();
    ts->tv_sec = static_cast<time_t>(ns / 1000000000LL);
    ts->tv_nsec = static_cast<long>(ns % 1000000000LL);
  }
  return rc;
}

int main() {
  std::printf("[SYNTHETIC] process-local CLOCK_REALTIME injection; "
              "OS clock / driver / hardware untouched\n");

  // Production behaviour: anchors are captured on the first HostNowNs() call,
  // i.e. before any injected step (as the real driver does at first packet).
  const std::uint64_t host0 = livox_ros::HostNowNs();
  const long long wall0 = WallNowNs();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // 1) Backstep by -600 ms (synthetic; same order of magnitude as the real
  //    -598.3 ms incident).
  wall_offset_ns.store(-600000000LL);
  const std::uint64_t host1 = livox_ros::HostNowNs();
  const long long wall1 = WallNowNs();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // 2) Forward step: from -600 ms to +700 ms (+1300 ms), matching Codex's
  //    +1310.063 ms measured wall delta.
  wall_offset_ns.store(700000000LL);
  const std::uint64_t host2 = livox_ros::HostNowNs();
  const long long wall2 = WallNowNs();

  const long long back_delta = wall1 - wall0;
  const long long fwd_delta = wall2 - wall1;
  const std::uint64_t host_d1 = host1 - host0;
  const std::uint64_t host_d2 = host2 - host1;

  std::printf("backstep:    CLOCK_REALTIME delta %+.3f ms, HostNowNs delta %+.3f ms\n",
              static_cast<double>(back_delta) / 1e6, static_cast<double>(host_d1) / 1e6);
  std::printf("forwardstep: CLOCK_REALTIME delta %+.3f ms, HostNowNs delta %+.3f ms\n",
              static_cast<double>(fwd_delta) / 1e6, static_cast<double>(host_d2) / 1e6);

  bool ok = true;

  // Fault injection must be observable: a real backstep and a real forward step.
  const bool injected = back_delta <= -500000000LL && fwd_delta >= 1200000000LL;
  std::printf("process-local fault injection %s\n", injected ? "VERIFIED" : "FAILED");
  ok = ok && injected;

  // Production mapping must stay monotonic and only advance by ~the 10 ms sleep.
  const std::uint64_t max_step_ns = 100000000ULL;  // 100 ms tolerance
  const bool monotonic = host1 > host0 && host2 > host1;
  const bool stable = monotonic && host_d1 < max_step_ns && host_d2 < max_step_ns;
  std::printf("production HostNowNs mapping %s "
              "(strictly increasing=%s, deltas within 100 ms=%s)\n",
              stable ? "PASS" : "FAIL", monotonic ? "yes" : "no",
              (host_d1 < max_step_ns && host_d2 < max_step_ns) ? "yes" : "no");
  ok = ok && stable;

  std::printf(ok ? "\nALL CHECKS PASSED\n" : "\nCHECKS FAILED\n");
  return ok ? 0 : 1;
}
