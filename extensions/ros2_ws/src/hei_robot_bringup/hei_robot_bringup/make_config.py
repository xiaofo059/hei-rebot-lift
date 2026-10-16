#!/usr/bin/env python3
"""生成 MID-360S 现场配置 JSON。

设计要点（对应任务 HEI-LIDAR-20261016-02-PREP 步骤 e）：
  * ``--host-ip`` / ``--lidar-ip`` 为**必填**，且没有任何示例 IP 默认值；
  * IPv4 校验：拒绝非法、组播、未指定(0.0.0.0)、广播、环回，以及 host == lidar；
  * 结构为官方 MID360s（``Mid360s`` 顶层键 + 数组 ``host_net_info`` + 官方端口）；
  * 原始点云外参保持 identity（不是 base/imu 实测外参）；
  * 只允许写入被 git 忽略的目录（默认建议 ``extensions/config/local/``）；
  * **不覆盖已有文件**：目标存在即拒绝，需人工确认后再处理。

退出码：0 成功；2 输入/校验失败；3 拒绝写入（已存在或未被忽略）。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from hei_robot_bringup.mid360s_config import (
    ConfigError,
    build_config,
    dumps_config,
)

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_REFUSED = 3

#: 推荐输出位置（已被根 .gitignore 的 /extensions/config/local/ 忽略）
SUGGESTED_OUTPUT = "extensions/config/local/mid360s_config.json"


def _git(args: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        check=False,
    )


def _nearest_existing_dir(path: Path) -> Path:
    """向上找到第一个真实存在的祖先目录。

    首次运行时 ``extensions/config/local/`` 可能整条路径都不存在；
    如果直接把不存在的目录交给 ``subprocess(cwd=...)``，会抛
    ``FileNotFoundError``（RF-LIDAR-001 复现的缺陷）。
    """
    current = path
    while not current.exists():
        parent = current.parent
        if parent == current:  # 到达文件系统根
            return current
        current = parent
    return current


def ensure_ignored(target: Path) -> None:
    """确认目标文件位于被 git 忽略的位置。

    现场配置含机器地址，属于本机值，不应进入共享仓库；这里用 git 自己的
    判定（``git check-ignore``）而不是复制一份路径白名单，避免规则漂移。
    若不在任何 git 工作树内（例如临时目录、测试环境）则跳过该检查。
    """
    directory = _nearest_existing_dir(target.parent)
    inside = _git(["rev-parse", "--is-inside-work-tree"], directory)
    if inside.returncode != 0 or inside.stdout.strip() != "true":
        print(
            f"提示：{directory} 不在 git 工作树内，跳过忽略检查。",
            file=sys.stderr,
        )
        return

    root = _git(["rev-parse", "--show-toplevel"], directory)
    repo_root = Path(root.stdout.strip()) if root.returncode == 0 else directory
    try:
        rel = target.resolve().relative_to(repo_root.resolve())
    except ValueError:
        # 目标在仓库外，交给上面的“非工作树”语义处理不了，按不在仓库内放行
        print(f"提示：{target} 在仓库 {repo_root} 之外，跳过忽略检查。", file=sys.stderr)
        return

    checked = _git(["check-ignore", "-q", "--", str(rel)], repo_root)
    if checked.returncode != 0:
        raise ConfigError(
            f"拒绝写入未被 git 忽略的路径：{target}\n"
            "  现场配置含机器地址，不应提交。请改用被忽略的目录，例如：\n"
            f"    {SUGGESTED_OUTPUT}\n"
            "  （根 .gitignore 已忽略 /extensions/config/local/ 与 /extensions/third_party/）"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mid360s_make_config",
        description="生成 MID-360S 现场配置 JSON（不含任何示例 IP 默认值）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例（把地址换成现场实测值）：\n"
            "  ros2 run hei_robot_bringup mid360s_make_config \\\n"
            "      --host-ip <本机有线网卡IP> \\\n"
            "      --lidar-ip <雷达IP> \\\n"
            f"      --output {SUGGESTED_OUTPUT}\n"
        ),
    )
    parser.add_argument("--host-ip", required=True, help="本机有线网卡 IPv4（必填，无默认值）")
    parser.add_argument("--lidar-ip", required=True, help="MID-360S 雷达 IPv4（必填，无默认值）")
    parser.add_argument(
        "--output",
        required=True,
        help=f"输出 JSON 路径（必须位于被 git 忽略的目录，建议 {SUGGESTED_OUTPUT}）",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="只在标准输出打印配置，不写文件（用于人工核对）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = build_config(args.host_ip, args.lidar_ip)
    except ConfigError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_USAGE

    text = dumps_config(config)

    if args.print_only:
        sys.stdout.write(text)
        return EXIT_OK

    target = Path(args.output).expanduser()
    if target.exists():
        print(
            f"错误：目标已存在，拒绝覆盖：{target}\n"
            "  本工具不会覆盖已有现场配置。请先人工确认该文件用途，"
            "再改名备份或改用其他路径。",
            file=sys.stderr,
        )
        return EXIT_REFUSED

    try:
        ensure_ignored(target)
    except ConfigError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return EXIT_REFUSED

    target.parent.mkdir(parents=True, exist_ok=True)
    # 先以独占方式创建，避免与并发写入互相覆盖
    try:
        with target.open("x", encoding="utf-8") as handle:
            handle.write(text)
    except FileExistsError:
        print(f"错误：目标已存在（并发创建），拒绝覆盖：{target}", file=sys.stderr)
        return EXIT_REFUSED

    print(f"已写入现场配置：{target}")
    print(f"  host_ip  = {config['Mid360s']['host_net_info'][0]['host_ip']}")
    print(f"  lidar_ip = {config['lidar_configs'][0]['ip']}")
    print(f"  S 型键   = 'Mid360s'（数组 host_net_info，官方端口）")
    print("下一步（不要在本任务中连接雷达）：")
    print("  ros2 launch hei_robot_bringup mid360s_bringup.launch.py \\")
    print(f"      config_path:={target}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
