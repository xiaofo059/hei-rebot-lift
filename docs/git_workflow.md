# HEI ReBot Lift Git Workflow

[English](git_workflow.md) | [中文](git_workflow_zh.md)

This document describes the branch model, dual-machine setup, and daily commit flow
used for reproducing and developing HEI ReBot Lift. The goal is to keep the code and
the robot host/client protocol consistent while developing on both the robot host
(Jetson AGX) and the laptop (Ubuntu 22.04), avoiding hidden version mismatches.

## Branch Model

| Branch | Purpose | Machine |
| --- | --- | --- |
| `main` | Stable line that follows the upstream `lipengdong/hei-rebot-lift` | Usually no direct development |
| `dev` | Development mainline, checked out on both machines | Both machines |
| `feature/*` | Feature branch, branched from `dev` and merged back into `dev` | As needed |

```text
main (stable, follows upstream lipengdong/hei-rebot-lift)
 └── dev (development mainline, checked out on both machines)
      ├── feature/xxx
      └── feature/yyy
```

## Dual-Machine Split (One Repository, Different Directories)

The robot host and the laptop use the same codebase but different directories:

| Machine | Development directory | Responsibility |
| --- | --- | --- |
| Robot host (Jetson) | `src/lerobot/robots/hei_rebot_lift/`, `src/lerobot/motors/damiao_u2can/` | Hardware drivers, robot host, serial binding |
| Laptop (Ubuntu 22.04) | `examples/hei_rebot_lift/VR_mujoco_ik/`, teleoperation/training scripts | MuJoCo, IK, VR teleoperation, recording and training |

The two sides are tightly coupled through the ZMQ action/observation schema, so any
protocol change must keep both machines on the same branch.

## One-Time Setup (Run on Each Machine)

Git configuration is per-clone, so run it once on both the robot host and the laptop.

### 1. Clone and Remotes

```bash
git clone https://github.com/xiaofo059/hei-rebot-lift.git
cd hei-rebot-lift

git remote add upstream https://github.com/lipengdong/hei-rebot-lift.git

git fetch origin
git checkout -b dev origin/dev
```

If you already have a clone, skip the clone and just add upstream and create the
`dev` branch.

### 2. Safety Configuration

```bash
git config pull.ff only          # pull only fast-forwards; error on divergence
git config pull.rebase false     # never auto-rebase
git config fetch.prune true      # prune deleted remote branches on fetch
git config push.default simple   # push only the current branch
git config push.autoSetupRemote true   # auto-track on first push of a new branch
git config alias.lg 'log --oneline --graph --decorate --all -20'
git config alias.sync '!git fetch upstream && git fetch origin --prune'
```

## Daily Workflow

### Sync Upstream Updates

```bash
git sync                       # fetch latest from upstream + origin
git checkout main
git merge upstream/main        # merge upstream updates into main
git push origin main           # push back to the fork
git checkout dev
git merge main                 # sync into dev
```

### Develop a Feature

```bash
git checkout dev
git pull
git checkout -b feature/name
# develop, test...
git add .
git commit -m "description"
git push                       # first push auto-creates tracking
```

### Merge Back into dev

```bash
git checkout dev
git pull
git merge feature/name
git push
git branch -d feature/name
```

See the full picture with: `git lg`

## Cross-Machine Conventions

- When changing the ZMQ protocol, action/observation fields, camera names, or joint
  dimensions, keep both the robot host and the laptop **on the same feature branch**
  during development, then merge back into `dev` after integration testing.
- `dev` is a shared branch; never rebase or rewrite its history. It is already
  configured with `pull.rebase=false` + `pull.ff=only`.
- Keep only one robot command source at a time; this is independent of code versions
  and also applies during deployment and debugging.

## Notes

- Pure-simulation laptop work that does not change the protocol (IK tuning, URDF
  changes, VR display tweaks) can use a separate branch freely.
- When upstream has new commits, merge them into `main` first, then sync into `dev`;
  do not pull upstream directly on `dev`.
- Source code and `environment.yml` take precedence over documented parameters.
