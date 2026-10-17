// HEI-LIDAR-20261017-02-FIX: no-hardware reproducer for the SDK exit crash.
//
// Originally authored by Codex (2026-10-17) at
// /home/robo/hei-data/mid360s_retest_20261017_evidence/probe_sdk_loader.cpp and
// kept here as a tracked, reproducible test. It loads the SDK and calls
// LivoxLidarSdkInit(nullptr, nullptr, nullptr): the frozen source runs
// InitLogger() first and then returns false, without opening any device or
// socket. Natural process teardown then runs the SDK static spdlog logger
// destructor.
//
//   original SDK alone                 -> exit 0
//   dlopen(system libspdlog) + SDK     -> SIGBUS(-7)  (before the symbol patch)
//   dlopen(system libspdlog) + patched -> exit 0      (expected after the patch)
//
// Compile: g++ -O2 -o probe_sdk_loader probe_sdk_loader.cpp -ldl
// Run (second arg requests system libspdlog first):
//   ulimit -c 0; ./probe_sdk_loader <liblivox_lidar_sdk_shared.so> [--system-first]
#include <dlfcn.h>
#include <cstdio>

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  // Resolve by SONAME through the dynamic loader search path instead of a
  // hardcoded x86_64 path. This also works on aarch64 distributions that ship
  // libspdlog.so.1; the ARM64 case is intended but NOT tested here.
  if (argc > 2 && !dlopen("libspdlog.so.1", RTLD_NOW | RTLD_GLOBAL)) {
    std::puts(dlerror());
    return 3;
  }
  void* h = dlopen(argv[1], RTLD_NOW | RTLD_GLOBAL);
  if (!h) {
    std::puts(dlerror());
    return 4;
  }
  auto init = reinterpret_cast<bool (*)(const char*, const char*, const void*)>(
      dlsym(h, "LivoxLidarSdkInit"));
  if (!init) {
    std::puts(dlerror());
    return 5;
  }
  // Frozen source checks the null arguments AFTER InitLogger(), BEFORE any
  // socket/device initialisation.
  bool rc = init(nullptr, nullptr, nullptr);
  std::printf("Init(null,null,null)=%d; normal process teardown follows\n", int(rc));
  std::fflush(stdout);
  return rc ? 6 : 0;
}
