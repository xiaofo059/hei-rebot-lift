# driver 退出崩溃：符号/ODR 证据与修复后验收

## 1. 版本与载入路径

| 组件 | 版本/位置 | 证据 |
| --- | --- | --- |
| SDK 内置 spdlog | **1.3.1**（header-only） | `extensions/third_party/livox_sdk2_src/3rdparty/spdlog/spdlog/version.h:8-10` |
| 系统 spdlog | **1.9.2**（`libspdlog.so.1`） | `dpkg -l libspdlog1` = `1:1.9.2+ds-0.2`；`ldconfig -p` |
| ROS 日志后端 | `librcl_logging_spdlog.so` | `readelf -d` → `NEEDED libspdlog.so.1`；rclcpp 初始化时载入 |
| SDK shared lib | `liblivox_lidar_sdk_shared.so` | 打包 spdlog/fmt 头文件代码（SDK `sdk_core/CMakeLists.txt:45` 仅加 include 目录） |

## 2. 修复前：符号表冲突（默认可见性弱符号）

```
$ nm -D --defined-only liblivox_lidar_sdk_shared.so | grep -c spdlog   # 421
$ nm -D --defined-only liblivox_lidar_sdk_shared.so | grep -c 3fmt     # 124
符号类型分布： 8 u / 149 V / 264 W（全部 default visibility 导出）

两个库定义同一个弱模板符号：
  SDK        : 000...557b0 W _ZNSt23_Sp_counted_ptr_inplaceIN6spdlog6loggerE...10_M_disposeEv
  libspdlog.1: 000...3b9d0 W _ZNSt23_Sp_counted_ptr_inplaceIN6spdlog6loggerE...10_M_disposeEv
```

运行时 libspdlog 先进入全局作用域 ⇒ SDK 内部对该符号的引用被抢占，
于是 1.3.1 构造的 logger 由 1.9.2 的 `_M_dispose` 析构（coredump 栈见 `coredump_344190_sigbus.txt`）。

## 3. 修复：shared 库只导出公开 C API

patch：`extensions/dependencies/patches/livox_sdk2/0001-export-only-public-c-api.patch`

- 新增 `sdk_core/livox_lidar_sdk_shared.map`：`global:` 仅列 `livox_lidar_api.h` 的 56 个函数，
  `local: *;` 隐藏 spdlog/fmt/livox:: 内部符号；
- `sdk_core/CMakeLists.txt`：`LINK_FLAGS -Wl,--version-script=...` + `LINK_DEPENDS`（只改 map 也重链）。

修复后（构建脚本内置验收）：

```
OK: 未导出 spdlog/fmt/livox:: 内部符号
OK: 公开 C API 导出完整（56 个）
```

## 4. 无硬件 A/B 探针（因果实验）

探针：`extensions/tools/tests/probe_sdk_loader.cpp`（Codex 原始实现的可追踪副本）。
只 `dlopen` + `LivoxLidarSdkInit(nullptr,nullptr,nullptr)`（冻结源码先 InitLogger 后返回 false，
不打开设备/套接字），自然进程退出即触发 SDK 全局 logger 析构。

| 场景 | 修复前（原 SDK） | 修复后（新 SDK） |
| --- | --- | --- |
| SDK 单独加载 | 0 | 0 |
| 先 `dlopen` 系统 libspdlog(RTLD_GLOBAL) 再加载 SDK | **SIGBUS(135)** | **0** |

命令：

```bash
LIVOX_ORIGINAL_SDK=/home/robo/hei-data/mid360s_retest_20261017_evidence/original_sdk.so \
  extensions/tools/tests/run_underlay_regression.sh
```

结果：`ALL UNDERLAY REGRESSION CHECKS PASSED`（完整输出见 `underlay_regression.log`）。
