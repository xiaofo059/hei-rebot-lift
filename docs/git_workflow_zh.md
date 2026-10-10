# HEI ReBot Lift Git 工作流

[English](git_workflow.md) | [中文](git_workflow_zh.md)

本文说明 HEI ReBot Lift 复刻与开发使用的分支模型、双机配置和日常提交流程。目标是让
机器人端（Jetson AGX）与笔记本端（Ubuntu 22.04）同时开发时，代码与机器人 host/client
协议保持一致，避免隐性版本错配。

## 核心约束

1. **源码只读**：fork 仓库中来自母项目的**源码**（`software/`、`hardware/`、`docs/`、
   `community/`、`media/` 等）保持只读，不做任何功能修改。
2. **母项目更新是源码变化的唯一来源**：只有母项目作者（`lipengdong/hei-rebot-lift`）
   更新后，你拉取更新（`main` 同步 upstream，再同步进 `dev`）。
3. **运行期生成文件与采集数据允许写入**：运行代码产生的必要文件（如设备映射的 udev
   规则 `99-nx-robot.rules`）、采集的数据集、训练输出等可写入仓库对应目录，不算改动源码。
4. **新功能另建工作空间**：开发新功能（如雷达导航、新增传感器等）时，一律在**仓库之外
   的独立文件夹（工作空间）**中开发，不修改仓库内母项目源码。

```text
hei-rebot-lift/              # fork 仓库：源码只读，运行期生成文件/数据可写
    main（同步 upstream）
    dev（同步 main）
hei-rebot-workspace/         # 独立工作空间：新功能开发（如雷达导航，可单独 git init）
```

## 分支模型

| 分支 | 用途 | 说明 |
| --- | --- | --- |
| `main` | 只同步母项目 `lipengdong/hei-rebot-lift` 的更新 | 不在上面开发 |
| `dev` | 同步 `main` 的母项目更新；承载工作流文档等少量维护文件 | 两台机器都 checkout |

```text
main（仅同步上游 lipengdong/hei-rebot-lift）
 └── dev（同步 main；仓库代码保持只读）

新功能不进入仓库分支，而是放到独立工作空间目录：
hei-rebot-workspace/（另建文件夹，可单独 git init）
```

## 双机分工（同一仓库，不同目录）

机器人端与笔记本端使用同一份代码，但开发目录不同：

| 机器 | 开发目录 | 职责 |
| --- | --- | --- |
| 机器人端 Jetson | `src/lerobot/robots/hei_rebot_lift/`、`src/lerobot/motors/damiao_u2can/` | 硬件驱动、机器人 host、串口绑定 |
| 笔记本 Ubuntu 22.04 | `examples/hei_rebot_lift/VR_mujoco_ik/`、遥操作/训练脚本 | MuJoCo、IK、VR 遥操作、录制与训练 |

两者通过 ZMQ 动作/观测字段强耦合，两台机器必须保持同一上游版本。

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

### 开发新功能（在独立工作空间，不改仓库代码）

新功能不修改仓库内任何母项目代码，而是在仓库之外另建工作空间目录：

```bash
# 在仓库同级（或任意位置）另建工作空间
mkdir -p ~/hei-rebot-workspace
cd ~/hei-rebot-workspace
git init                        # 需要版本管理时单独初始化，独立于 fork 仓库
# 在这里编写新功能代码；需要时可引用仓库代码，但不修改仓库本身
```

> 仓库（`hei-rebot-lift/`）只做上游同步，不承担新功能代码；新功能代码、配置与说明
> 都放在独立工作空间目录中，方便协作与回溯。

查看仓库全貌：`git lg`

## 跨端约定

- 机器人端与笔记本端应保持 fork 仓库在**同一上游版本**（同一提交），避免 host/client 协议不一致。
- 协议相关改动只来自母项目更新；更新后两台机器同步到同一版本再联调。
- `dev` 是共享分支，禁止在上面直接 rebase 或改历史；已配置 `pull.rebase=false` + `pull.ff=only`。
- 同一时间只保留一个机器人控制源，与代码版本无关，部署与调试时同样遵守。

## 注意事项

- 仓库源码只读，功能开发（如雷达导航）放到独立工作空间目录，禁止直接改母项目源码。
- 运行期生成文件与采集数据允许写入：udev 规则、数据集、训练输出等；这些多为
  `.gitignore` 已忽略，或属于设备绑定的必要文件。
- 母项目有新提交时，先合并到 `main` 再同步进 `dev`，不要在 `dev` 上直接 pull 上游。
- 代码与 `environment.yml` 优先于文档参数。
