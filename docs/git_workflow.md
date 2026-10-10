# HEI ReBot Lift Git Workflow

[English](git_workflow.md) | [中文](git_workflow_zh.md)

This document describes the branch model, dual-machine setup, and daily commit flow
used for reproducing and developing HEI ReBot Lift. The goal is to keep the code and
the robot host/client protocol consistent while developing on both the robot host
(Jetson AGX) and the laptop (Ubuntu 22.04), avoiding hidden version mismatches.

## Core Constraints (Must Follow)

1. **Repository code is read-only**: all code from the upstream project inside the fork
   (`software/`, `hardware/`, `docs/`, `community/`, `media/`, etc.) stays read-only;
   do not modify it.
2. **The only source of repository changes**: when the upstream author
   (`lipengdong/hei-rebot-lift`) updates the project, you pull the updates (`main`
   syncs upstream, then `dev` syncs from `main`). Do not edit repository code otherwise.
3. **New features live in a separate workspace**: robot feature development always
   happens in a **separate folder (workspace) outside the repository**, never by
   editing the repository code directly. The workspace may reference repository code,
   but feature code is stored and managed independently; initialize a separate git
   repository for the workspace when version control is needed.

```text
hei-rebot-lift/              # fork repository: read-only, only syncs upstream
    main (syncs upstream)
    dev (syncs main)
hei-rebot-workspace/         # separate workspace: feature development (optional git init)
```

## Branch Model

| Branch | Purpose | Note |
| --- | --- | --- |
| `main` | Only syncs updates from upstream `lipengdong/hei-rebot-lift` | No development here |
| `dev` | Syncs upstream updates from `main`; carries a few maintenance files like workflow docs | Checked out on both machines |

```text
main (only syncs upstream lipengdong/hei-rebot-lift)
 └── dev (syncs main; repository code stays read-only)

New features do not enter repository branches; they go into a separate workspace:
hei-rebot-workspace/ (a separate folder, optional git init)
```

## Dual-Machine Split (One Repository, Different Directories)

The robot host and the laptop use the same codebase but different directories:

| Machine | Development directory | Responsibility |
| --- | --- | --- |
| Robot host (Jetson) | `src/lerobot/robots/hei_rebot_lift/`, `src/lerobot/motors/damiao_u2can/` | Hardware drivers, robot host, serial binding |
| Laptop (Ubuntu 22.04) | `examples/hei_rebot_lift/VR_mujoco_ik/`, teleoperation/training scripts | MuJoCo, IK, VR teleoperation, recording and training |

The two sides are tightly coupled through the ZMQ action/observation schema, so both
machines must stay on the same upstream version.

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

### Develop a Feature (in a Separate Workspace, Without Editing the Repository)

New features never modify any upstream code in the repository; instead, create a
separate workspace folder outside the repository:

```bash
# create a workspace next to (or anywhere outside) the repository
mkdir -p ~/hei-rebot-workspace
cd ~/hei-rebot-workspace
git init                        # separate repository if version control is needed
# write feature code here; reference repository code if needed, but do not edit it
```

> The repository (`hei-rebot-lift/`) only syncs upstream and does not host feature
> code; feature code, configs and docs live in the separate workspace folder.

See the repository picture with: `git lg`

## Cross-Machine Conventions

- Keep the robot host and the laptop on the **same upstream version** (same commit) of
  the fork repository to avoid host/client protocol mismatches.
- Protocol changes only come from upstream updates; after an update, sync both machines
  to the same version before integration testing.
- `dev` is a shared branch; never rebase or rewrite its history. It is already
  configured with `pull.rebase=false` + `pull.ff=only`.
- Keep only one robot command source at a time; this is independent of code versions
  and also applies during deployment and debugging.

## Notes

- Repository code is read-only; put all feature changes in a separate workspace folder,
  never edit the upstream code inside the repository directly.
- When upstream has new commits, merge them into `main` first, then sync into `dev`;
  do not pull upstream directly on `dev`.
- The only allowed repository change is syncing upstream updates; a few maintenance
  files such as workflow docs are the exception.
- Source code and `environment.yml` take precedence over documented parameters.
