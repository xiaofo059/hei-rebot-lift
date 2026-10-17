"""配置生成 CLI 的无硬件测试：必填、IPv4 校验、不覆盖、忽略目录强制。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from hei_robot_bringup import make_config

GIT = "git"


def _run(*args: str) -> int:
    return make_config.main(list(args))


# --------------------------------------------------------------------------
# 必填与校验
# --------------------------------------------------------------------------


def test_missing_required_args_exits_usage(tmp_path):
    with pytest.raises(SystemExit) as excinfo:
        _run("--host-ip", "10.20.30.40", "--output", str(tmp_path / "c.json"))
    assert excinfo.value.code == 2  # --lidar-ip 必填


def test_no_example_ip_default_exists():
    """帮助文本里不能出现可被误抄的示例 IP。"""
    text = make_config.build_parser().format_help()
    for needle in ("192.168.", "10.0.0.", "172.16."):
        assert needle not in text


@pytest.mark.parametrize(
    "host_ip,lidar_ip",
    [
        ("999.1.1.1", "10.20.30.50"),
        ("10.20.30.40", "not-an-ip"),
        ("224.0.0.1", "10.20.30.50"),
        ("0.0.0.0", "10.20.30.50"),
        ("255.255.255.255", "10.20.30.50"),
        ("127.0.0.1", "10.20.30.50"),
        ("10.20.30.40", "10.20.30.40"),
    ],
)
def test_invalid_addresses_exit_usage_and_write_nothing(tmp_path, host_ip, lidar_ip):
    out = tmp_path / "cfg.json"
    assert _run("--host-ip", host_ip, "--lidar-ip", lidar_ip, "--output", str(out)) == 2
    assert not out.exists(), "校验失败时不得留下任何文件"


# --------------------------------------------------------------------------
# 成功路径
# --------------------------------------------------------------------------


def test_writes_config_with_official_structure(tmp_path, capsys):
    out = tmp_path / "mid360s.json"
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(out)) == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["Mid360s"]["host_net_info"][0]["host_ip"] == "10.20.30.40"
    assert data["lidar_configs"][0]["ip"] == "10.20.30.50"
    assert data["lidar_summary_info"]["lidar_type"] == 8
    assert "已写入现场配置" in capsys.readouterr().out


def test_print_only_writes_no_file(tmp_path, capsys):
    out = tmp_path / "cfg.json"
    assert _run(
        "--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50",
        "--output", str(out), "--print-only",
    ) == 0
    assert not out.exists()
    assert json.loads(capsys.readouterr().out)["Mid360s"]["host_net_info"][0]["host_ip"] == "10.20.30.40"


@pytest.mark.skipif(subprocess.run(["which", GIT], capture_output=True).returncode != 0,
                    reason="需要 git")
def test_first_run_creates_missing_multi_level_dirs(tmp_path):
    """RF-LIDAR-001 回归：首次运行时 extensions/config/local 整条路径都不存在。

    此前 ensure_ignored 会把不存在的 parent.parent 交给 subprocess(cwd=...)，
    直接抛 FileNotFoundError，配置生成在首次运行必然失败。
    """
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run([GIT, "init", "-q"], cwd=repo, check=True)
    (repo / ".gitignore").write_text("/extensions/config/local/\n", encoding="utf-8")

    target = repo / "extensions" / "config" / "local" / "mid360s_config.json"
    assert not target.parent.exists(), "前提：多级目录都不存在"

    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(target)) == 0
    assert target.is_file()
    assert json.loads(target.read_text(encoding="utf-8"))["Mid360s"]["host_net_info"][0][
        "host_ip"
    ] == "10.20.30.40"


@pytest.mark.skipif(subprocess.run(["which", GIT], capture_output=True).returncode != 0,
                    reason="需要 git")
def test_first_run_unignored_missing_dirs_refused_cleanly(tmp_path, capsys):
    """多级目录缺失且目标未被忽略时，必须干净拒绝（exit 3），不得崩栈。"""
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run([GIT, "init", "-q"], cwd=repo, check=True)
    (repo / ".gitignore").write_text("data/\n", encoding="utf-8")

    target = repo / "a" / "b" / "c" / "cfg.json"
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(target)) == 3
    assert not target.exists()
    assert "未被 git 忽略" in capsys.readouterr().err


def test_nearest_existing_dir_walks_up(tmp_path):
    from hei_robot_bringup.make_config import _nearest_existing_dir

    deep = tmp_path / "x" / "y" / "z"
    assert _nearest_existing_dir(deep) == tmp_path
    assert _nearest_existing_dir(tmp_path) == tmp_path
    (tmp_path / "x").mkdir()
    assert _nearest_existing_dir(deep) == tmp_path / "x"


# --------------------------------------------------------------------------
# 不覆盖已有文件
# --------------------------------------------------------------------------


def test_refuses_to_overwrite_existing_file(tmp_path, capsys):
    out = tmp_path / "cfg.json"
    out.write_text('{"keep": "me"}\n', encoding="utf-8")
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(out)) == 3
    assert out.read_text(encoding="utf-8") == '{"keep": "me"}\n', "已有文件必须原样保留"
    assert "拒绝覆盖" in capsys.readouterr().err


# --------------------------------------------------------------------------
# 只允许写入被忽略的位置
# --------------------------------------------------------------------------


@pytest.mark.skipif(subprocess.run(["which", GIT], capture_output=True).returncode != 0,
                    reason="需要 git")
def test_refuses_path_not_ignored_by_git(tmp_path, capsys):
    repo = tmp_path / "repo"
    repo.mkdir()
    subprocess.run([GIT, "init", "-q"], cwd=repo, check=True)
    (repo / ".gitignore").write_text("data/\n", encoding="utf-8")

    tracked = repo / "cfg.json"          # 未被忽略
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(tracked)) == 3
    assert not tracked.exists()
    assert "未被 git 忽略" in capsys.readouterr().err


@pytest.mark.skipif(subprocess.run(["which", GIT], capture_output=True).returncode != 0,
                    reason="需要 git")
def test_allows_path_inside_ignored_directory(tmp_path):
    repo = tmp_path / "repo"
    (repo / "extensions" / "config" / "local").mkdir(parents=True)
    subprocess.run([GIT, "init", "-q"], cwd=repo, check=True)
    (repo / ".gitignore").write_text("/extensions/config/local/\n", encoding="utf-8")

    target = repo / "extensions" / "config" / "local" / "mid360s.json"
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(target)) == 0
    assert target.is_file()


def test_allow_non_git_directory(tmp_path):
    """不在 git 工作树内（临时目录）时放行，但仍写文件。"""
    out = tmp_path / "plain" / "cfg.json"
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(out)) == 0
    assert out.is_file()


def test_generated_config_passes_its_own_validator(tmp_path):
    """生成与校验使用同一模块，闭环必须自洽。"""
    from hei_robot_bringup.mid360s_config import load_and_validate

    out = tmp_path / "cfg.json"
    assert _run("--host-ip", "10.20.30.40", "--lidar-ip", "10.20.30.50", "--output", str(out)) == 0
    field = load_and_validate(out)
    assert (field.host_ip, field.lidar_ip) == ("10.20.30.40", "10.20.30.50")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
