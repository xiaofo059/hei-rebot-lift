# HEI ReBot Lift Git 工作流

[English](git_workflow.md) | [中文](git_workflow_zh.md)

本文说明 HEI ReBot Lift 复刻与开发使用的分支模型、双机配置和日常提交流程。目标是让
机器人端（Jetson AGX）与笔记本端（Ubuntu 22.04）同时开发时，代码与机器人 host/client
协议保持一致，避免隐性版本错配。

## 分支模型

| 分支 | 用途 | 对应机器 |
| --- | --- | --- |
| `main` | 稳定线，跟随母项目 `lipengdong/hei-rebot-lift` | 一般不在上面直接开发 |
| `dev` | 开发主线，机器人端与笔记本端都 checkout 这个分支 | 两台机器 |
| `feature/*` | 新功能分支，从 `dev` 拉出，合并回 `dev` | 按需 |

```text
main（稳定，跟随上游 lipengdong/hei-rebot-lift）
 └── dev（开发主线，两台机器都 checkout）
      ├── feature/xxx
      └── feature/yyy
```

## 双机分工（同一仓库，不同目录）

机器人端与笔记本端使用同一份代码，但开发目录不同：

| 机器 | 开发目录 | 职责 |
| --- | --- | --- |
| 机器人端 Jetson | `src/lerobot/robots/hei_rebot_lift/`、`src/lerobot/motors/damiao_u2can/` | 硬件驱动、机器人 host、串口绑定 |
| 笔记本 Ubuntu 22.04 | `examples/hei_rebot_lift/VR_mujoco_ik/`、遥操作/训练脚本 | MuJoCo、IK、VR 遥操作、录制与训练 |

两者通过 ZMQ 动作/观测字段强耦合，涉及协议改动时必须保持同一分支。

## 一次性初始化（两台机器各自执行）

git 配置是每个克隆独立的，机器人端与笔记本端都要执行一次。

### 1. 克隆与 remote

```bash
git clone https://github.com/xiaofo059/hei-rebot-lift.git
cd hei-rebot-lift

git remote add upstream https://github.com/lipengdong/hei-rebot-lift.git

git fetch origin
git checkout -b dev origin/dev
```

已有克隆时跳过 clone，直接添加 upstream 并建立 dev 分支。

### 2. 安全配置

```bash
git config pull.ff only          # pull 只允许快进，发现分叉就报错
git config pull.rebase false     # 明确不自动 rebase
git config fetch.prune true      # fetch 自动清理已删除的远程分支
git config push.default simple   # 只推送当前分支到同名远程分支
git config push.autoSetupRemote true   # 新分支首次 push 自动建立跟踪
git config alias.lg 'log --oneline --graph --decorate --all -20'
git config alias.sync '!git fetch upstream && git fetch origin --prune'
```

## 日常工作流

### 同步母项目更新

```bash
git sync                       # 拉取 upstream + origin 最新
git checkout main
git merge upstream/main        # 把母项目更新合并进 main
git push origin main           # 推回 fork
git checkout dev
git merge main                 # 同步进 dev
```

### 开发新功能

```bash
git checkout dev
git pull
git checkout -b feature/功能名
# 开发、测试……
git add .
git commit -m "描述"
git push                       # 首次 push 自动建立跟踪
```

### 合并回 dev

```bash
git checkout dev
git pull
git merge feature/功能名
git push
git branch -d feature/功能名
```

查看全貌：`git lg`

## 跨端联调约定

- 涉及 ZMQ 协议、动作/观测字段、相机名称或关节维度改动时，开发期间机器人端与笔记本端
  **同时 checkout 同一个 feature 分支**，联调通过后再合并回 `dev`。
- `dev` 是共享分支，禁止在上面直接 rebase 或改历史；已配置 `pull.rebase=false` + `pull.ff=only`。
- 同一时间只保留一个机器人控制源，与代码版本无关，部署与调试时同样遵守。

## 注意事项

- 纯仿真、不改协议的笔记本工作（调 IK、改 URDF、改 VR 显示）可在笔记本开独立分支自由折腾。
- 母项目有新提交时，先合并到 `main` 再同步进 `dev`，不要在 `dev` 上直接 pull 上游。
- 代码与 `environment.yml` 优先于文档参数。
