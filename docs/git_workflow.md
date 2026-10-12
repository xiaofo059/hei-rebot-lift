# HEI ReBot Lift: Dual-Machine Development and Upstream Synchronization

[English](git_workflow.md) | [中文](git_workflow_zh.md)

This workflow supports reproducing the upstream project while developing MID360,
ROS 2 Humble navigation, and other capabilities on a Jetson AGX and a laptop.
The default is **one fork, one extension directory, packages organized by function,
feature branches for integration, and tested versions for deployment**.

These are collaboration conventions, not a blanket prohibition on editing upstream
code or a requirement to seek approval for every edit. Prefer public interfaces and
configuration; when those are insufficient, make small, testable upstream changes.
Avoid unrelated refactoring while adding a feature.

> The `extensions/` tree, ROS packages, motion gateway, and deployment records below
> are a proposed structure, not existing functionality. This documentation rewrite
> does not create directories, install dependencies, change branches, or migrate files.
> Execute commands by stage; replace placeholders first. Do not run all blocks as one script.

## 1. Repository and Branch Responsibilities

| Name | Purpose |
| --- | --- |
| `upstream` | Original project: `https://github.com/lipengdong/hei-rebot-lift.git` |
| `origin` | Your fork: `https://github.com/xiaofo059/hei-rebot-lift.git` |
| `main` | Follows `upstream/main`, without custom feature commits |
| `dev` | Long-lived integration branch: upstream updates, custom features, machine configuration, and documentation |
| `feat/<topic>` | Short-lived feature branch from `dev`, e.g. `feat/mid360-bringup` |
| `fix/<topic>` | Bug fix, e.g. `fix/base-command-timeout` |
| `sync/upstream-<date>` | Validation branch for integrating a newer `main` into `dev` |
| `robot-tested/<date>-<n>` tag | Fork commit that passed the recorded checks, for deployment and rollback |

```text
upstream/main → origin/main → sync/upstream-date → dev → tested tag
                                                 ↑
                                       feat/* and fix/*
```

Custom features belong in `dev`. Feature branches can be deleted after merging;
their source remains in `dev`. Neither each module nor each machine needs a permanent
branch. Small documentation fixes may go directly to `dev`; hardware interfaces,
navigation, and upstream upgrades should normally use separate branches.

Use merges and normal pushes for shared `main`, `dev`, and handed-off feature branches.
Avoid rewriting history that another machine or contributor relies on. Unshared
personal feature branches may be reorganized locally.

## 2. Where New Features Live

**Put custom development under `extensions/` at the root of this fork.** A single
commit can then identify the upstream baseline, custom code, configuration templates,
and deployment instructions. Directory boundaries reduce conflicts while keeping
features under the same version control.

```text
hei-rebot-lift/
├── software/lerobot-hei-rebot-lift/  # Upstream software; keep changes focused
├── hardware/, media/, docs/         # Existing resources and documentation
└── extensions/                     # Custom development; create as needed
    ├── README.md                   # Module index, startup order, machine, maturity
    ├── ros2_ws/                    # Custom ROS 2 overlay workspace
    │   ├── src/
    │   │   ├── hei_robot_interfaces/ # Custom msg/srv/action only when necessary
    │   │   ├── hei_robot_description/# Sensor transforms and model extensions
    │   │   ├── hei_robot_bridge/     # ROS ↔ gateway, units and state mapping
    │   │   ├── hei_robot_localization/ # Localization integration configuration
    │   │   ├── hei_robot_navigation/ # Nav2 parameters, behavior trees, launch
    │   │   ├── hei_robot_bringup/    # MID360/system launch and parameter assembly
    │   │   └── hei_robot_sim/        # Simulation adapter sharing real-robot interfaces
    │   └── build/, install/, log/   # Machine-local generated files; not committed
    ├── python/hei_motion_gateway/   # Separate Python package in the LeRobot environment
    ├── config/
    │   ├── common/                 # Shared defaults
    │   ├── jetson/                 # Robot-role templates
    │   ├── laptop/                 # Simulation/visualization-role templates
    │   └── local/                  # Untracked machine overrides
    ├── dependencies/               # Third-party .repos locks and SDK build instructions
    ├── third_party/                # Imported sources; ignored by the parent repository
    │   └── livox_ws/src/livox_ros_driver2/
    ├── scripts/                    # Repeatable checks, builds, and launch helpers
    ├── tests/                      # Cross-module, protocol, and integration tests
    └── docs/
        ├── decisions/              # Interface decisions and upstream patch rationale
        └── deployments/            # Machine versions and validation records
```

Do not create every empty package up front. Start with lidar configuration in
`hei_robot_bringup`, implement the bridge next, then integrate localization and navigation.
Ordinary Python tools belong in `extensions/python/`; they need not become ROS nodes.

For the first ROS package, enter a feature branch and prepare ROS 2 Humble, then run
from the repository root:

```bash
source /opt/ros/humble/setup.bash
mkdir -p extensions/ros2_ws/src
cd extensions/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 hei_robot_bringup
mkdir -p hei_robot_bringup/launch hei_robot_bringup/config
```

This creates a skeleton only. After adding launch/configuration files, install those
directories in `CMakeLists.txt`, declare runtime dependencies in `package.xml`, and add
a README. Creating directories does not implement lidar support.

The historical `software/ros_ws/` is an alternative location. Inventory any real code
before planning a migration; avoid maintaining two custom ROS workspaces indefinitely.
Handle migration or deletion as a separate, reviewable change rather than mixing it
into an upstream synchronization.

### Managing a Growing Set of Modules

- Give each package a clear responsibility. Declare ROS dependencies in `package.xml`
  and Python dependencies in `pyproject.toml`.
- Use one custom ROS workspace for multiple packages by default. Split workspaces for
  dependency conflicts, independent releases, or special build requirements.
- Each module README records purpose, execution machine, inputs/outputs, parameters,
  launch commands, and minimum validation steps.
- Maintain a module table in `extensions/README.md`: planned, experimental, or validated,
  plus dependencies.
- Prefer standard ROS messages; introduce `hei_robot_interfaces` only when necessary.
- Extract a separate repository when reuse, permissions, or release cadence justify it;
  pin its commit in a `.repos` manifest. Do not run nested `git init` for each custom package.

## 3. Code, Environments, and Configuration on Both Machines

| Item | Jetson AGX | Laptop |
| --- | --- | --- |
| Source | Clone the same fork | Clone the same fork |
| Normal role | Host, motors/cameras, MID360, navigation execution | Simulation, RViz, analysis, training, development |
| Integration role | Hardware and final command execution | Experimental algorithms or teleoperation client |
| ROS build | Native ARM64 build | Native x86_64 build, subject to actual hardware |
| Environments | LeRobot and ROS; host does not require VR | LeRobot, VR/IK, and ROS as needed |

Absolute checkout paths may differ. Unless stated otherwise, Git commands below run
from the local repository root. Synchronize source through Git, not whole-directory
copies. Do not transfer `build/`, `install/`, Conda environments, or caches between machines.

The bundled LeRobot uses Python 3.12; VR/IK uses Python 3.10. ROS 2 Humble deb packages
on Ubuntu 22.04 support amd64/arm64 and use the system Python 3.10. Check the Jetson's
exact model, JetPack/L4T, and Ubuntu version first: “AGX” does not identify its OS.
If incompatible, separately choose a supported OS approach or validate a container setup.
[Official ROS installation documentation](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/Ubuntu-Install-Debs.rst)

Run ROS, LeRobot, and VR in separate terminals/processes. Do not activate the LeRobot
Conda environment in ROS terminals or try to bridge them by installing `rclpy` into
Python 3.12. Use an explicit IPC protocol across environments.

Load configuration as shared defaults → machine role → local overrides. This is a
convention for new launchers to implement, not an existing upstream capability.
Commit templates; keep machine-specific IPs, interfaces, device mappings, and lidar
serial numbers in `config/local/`. Record configuration versions or checksums for
deployments. Back up calibration separately; “not committed” must not mean “only one copy.”
Changes to already tracked udev files still appear in Git.

For cross-machine ROS tests, align `ROS_DOMAIN_ID`, compatible RMW/QoS settings, network
reachability, and clocks. Configure the lidar subnet/IP, DDS discovery, and HEI ZMQ
addresses separately; matching Domain IDs alone does not establish connectivity.
Use actual time on hardware and consistent `use_sim_time` plus `/clock` in simulation/bag replay.

## 4. Integrating MID360, Localization, and Navigation

This is an extension architecture to implement and validate in stages. A working
lidar driver alone is not a navigation stack.

```text
MID360 → Livox ROS 2 driver → point cloud/IMU → localization/mapping → Nav2
                                                                      ↓ velocity request
ROS bridge → motion gateway (LeRobot environment) → existing host → chassis
                       ↑
             observations, ownership, stopping
```

### Stage 1: Sensor Data and Transforms

Keep custom launch/parameters in `hei_robot_bringup` and sensor extrinsics in
`hei_robot_description`. Verify networking, point cloud/IMU rates, timestamps,
axis conventions, packet loss, and RViz output before recording a short rosbag.

Use the official `livox_ros_driver2` and its required Livox-SDK2, recording both versions.
The official `build.sh humble` clears workspace build/install directories and assumes
a particular source layout. Isolate it at
`extensions/third_party/livox_ws/src/livox_ros_driver2/`, build according to the pinned
version's instructions, and use it as an underlay for custom packages. Do not run that
script inside a workspace containing all your custom packages.
[Driver documentation](https://github.com/Livox-SDK/livox_ros_driver2) · [Build script](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/build.sh)

### Stage 2: Bridge and Verifiable Chassis Control

`hei_robot_bridge` accepts ROS velocity requests; a separate gateway in the LeRobot
environment connects to the host using the existing client. Define and test IPC
message versions, timestamps/expiry, units, state, control modes, and errors before hardware use.

Three facts in the current source affect this design:

1. `HeiRebotLift.send_action()` defaults missing arm joint fields to `0.0`.
   **Do not send only three chassis fields converted from `cmd_vel`.** After acquiring
   fresh feedback and explicit control ownership, the gateway should latch holding
   targets for arms/grippers/lift and assemble complete 18-dimensional actions.
   Missing feedback must prevent arming, not cause zero-filled commands.
2. Chassis `x.vel/y.vel/theta.vel` use driver scaling parameters and are not directly
   interchangeable with ROS m/s and rad/s. Convert using wheel size, geometry, signs,
   and measured calibration. Unconverted feedback is not valid `/odom`.
3. Host observations use ZMQ PUSH/PULL, not broadcast. Multiple clients split messages.
   In navigation mode the gateway owns the host client connection and republishes state
   to ROS; do not attach another client as a passive monitor.

Initially use mutually exclusive modes: stop upstream teleoperation, recording, and
rollout clients before navigation; stop the navigation gateway before returning to
upstream workflows. Simultaneous navigation and arm control requires routing requests
through one arbiter and one complete-action sender. The upstream host does not currently
provide that arbitration.

Source references: [action assembly and chassis conversion](../software/lerobot-hei-rebot-lift/src/lerobot/robots/hei_rebot_lift/hei_rebot_lift.py)
and [host transport and watchdog](../software/lerobot-hei-rebot-lift/src/lerobot/robots/hei_rebot_lift/hei_rebot_lift_host.py).
Recheck these behaviors after upstream upgrades.

Reject expired requests. Navigation stop, input timeout, mode changes, and disconnection
must revoke velocity requests and stop the chassis. Never keep forwarding stale velocity,
which would defeat the host watchdog. That watchdog stops chassis and lift; it is not
a complete arm emergency-stop mechanism.

### Stage 3: Localization and Navigation

Establish reliable odometry and the TF chain `map → odom → base_link → lidar`, assigning
one publisher per transform. Select and validate localization compatible with Humble
and MID360 data on recordings/simulation before hardware integration. Decide whether
navigation consumes projected LaserScan data or a point-cloud obstacle layer; configure
height filtering, footprint, and blind spots. Localization and obstacle avoidance are
separate consumers.

Choose/configure Nav2 local control and velocity limits for the omnidirectional base,
including lateral motion. Do not assume differential-drive defaults apply. Validate in
order: read-only sensor data → fake host/simulation bridge → raised-wheel direction and
stop checks → low-speed localization → limited-area navigation. These are acceptance
targets, not claims of an existing navigation implementation.
See [Nav2 state estimation](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)
for TF responsibilities. This rolling documentation is an architectural reference;
check concrete plugins and parameters against the installed Humble version.

## 5. Dependencies, Builds, and Generated Files

Track custom source in the fork. Record third-party URLs and exact commit SHAs in
`extensions/dependencies/*.repos` and import them into ignored `extensions/third_party/`.
A moving branch such as `main/master` is insufficient for deployment reproducibility.
[Official vcstool documentation](https://github.com/dirk-thomas/vcstool)

Example `livox.repos` structure; replace the placeholder with a verified SHA first:

```yaml
repositories:
  livox_ws/src/livox_ros_driver2:
    type: git
    url: https://github.com/Livox-SDK/livox_ros_driver2.git
    version: REPLACE_WITH_VERIFIED_COMMIT_SHA
```

After installing vcstool, import from the repository root. Pin Livox-SDK2 similarly
and record its separate build/install procedure:

```bash
mkdir -p extensions/third_party
vcs validate < extensions/dependencies/livox.repos
vcs import extensions/third_party < extensions/dependencies/livox.repos
```

A `.repos` file pins source only; it does not install SDKs, system libraries, or Python
dependencies. Commit third-party modifications to your own driver fork and update the
manifest rather than leaving the only changes in an ignored directory. Validate dependency
upgrades on a separate feature branch.

When creating the workspace, add these rules to the **root `.gitignore`**. They are
proposed configuration, not changes made by this documentation rewrite:

```gitignore
/extensions/ros2_ws/build/
/extensions/ros2_ws/install/
/extensions/ros2_ws/log/
/extensions/third_party/
/extensions/config/local/
```

Commit source, `package.xml`, launch files, parameter templates, manifests, and tests.
Store bags, datasets, weights, and large maps outside the repository, e.g. `~/hei-data/`;
record their purpose, version, checksum, and retrieval location in Git. Small reusable
test maps can be committed or managed with LFS. Avoid mixing data and source with `git add .`.

The following build example assumes Humble, colcon, and rosdep are installed, rosdep
is initialized/updated, and required third-party underlays are already built according
to their own documentation. Open a fresh terminal without Conda or the custom overlay sourced:

```bash
# Start at the local repository root; omit Livox sourcing for early packages without that dependency
source /opt/ros/humble/setup.bash
source extensions/third_party/livox_ws/install/local_setup.bash
cd extensions/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

For a module and its workspace dependencies, use
`colcon build --symlink-install --packages-up-to hei_robot_navigation`.
In a new terminal, source ROS → third-party underlay → custom overlay before running
launch commands from the module README. Do not repeatedly build in a terminal that
already sourced an old overlay.
[Official ROS workspace tutorial](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.rst)

## 6. Git Setup and Daily Feature Development

Both machines use the same origin/upstream. Existing clones need not be recreated.
Inspect actual state first:

```bash
git status --short --branch
git remote -v
git branch -vv
```

Only if upstream is missing, run
`git remote add upstream https://github.com/lipengdong/hei-rebot-lift.git`.
Recommended per-clone configuration:

```bash
git config pull.ff only
git config pull.rebase false
git config fetch.prune true
git config push.default simple
```

Start the following operations with a clean worktree. Review and commit existing
changes on their appropriate branch or explicitly stash them. Do not discard changes
automatically or resolve divergence with `reset --hard`.

Example lidar feature:

```bash
git fetch origin --prune
git switch dev
git merge --ff-only origin/dev
git switch -c feat/mid360-bringup
# Create the needed extension files and validate this stage
# Stage explicit feature paths; inspect untracked files and the diff too
# Example, once files exist: git add extensions/ros2_ws/src/hei_robot_bringup
git diff --cached
# After reviewing staged changes:
git commit -m "feat: add MID360 bringup configuration"
git push -u origin feat/mid360-bringup
```

Prefer a PR targeting your fork's `dev`, describing behavior, machine impact, and
validation. Solo development can also use a local merge followed by a normal push.
If the remote advanced, fetch/merge and retest rather than force-pushing over it.

### Continuing on the Other Machine

Hand off by feature branch: commit and push on the first machine, then fetch on the second:

```bash
git fetch origin
# Only when creating the local tracking branch for the first time:
git switch --track origin/feat/mid360-bringup
# Subsequently use git switch feat/mid360-bringup, then:
git pull --ff-only
```

Prefer separate branches such as `feat/mid360-driver` and `feat/mid360-rviz` when both
machines work concurrently. If a shared branch diverges, inspect
`git log --left-right --oneline HEAD...@{u}`, merge and validate at one integration
location, then fast-forward the other. Editing Jetson files over SSH is also valid;
build and execute them in the Jetson environment.

## 7. Following Upstream Updates

An upstream update need not be deployed immediately. Review it on the laptop or an
isolated checkout, focusing on host/client protocol, Python dependencies, camera fields,
control parameters, and startup behavior. Use one machine to integrate each update;
the others obtain the result from origin.

First update the upstream-only branch. These commands assume local `main` and `dev` exist:

```bash
git fetch origin --prune
git fetch upstream --prune
git switch main
git merge --ff-only origin/main
git merge --ff-only upstream/main
git push origin main
```

If any fast-forward fails, stop and inspect the divergence. Do not merge custom features
into `main` or force-push an unchecked mirror. Normally local `main` now matches the
fetched `upstream/main`.

Integrate `main` on a temporary branch, choosing a fresh name for each update:

```bash
git switch dev
git merge --ff-only origin/dev
git switch -c sync/upstream-20261012
git merge --no-ff main
# Conflicts: resolve each file, git add explicit paths, then git merge --continue
# To cancel an unfinished merge: git merge --abort
# Publish after protocol, module, and simulation checks
git push -u origin sync/upstream-20261012
```

Merge through a PR targeting `dev`. For local integration, update `dev` first, run
`git merge --no-ff sync/upstream-20261012`, validate the final result, and
`git push origin dev`. If `dev` advances during validation, merge it into the integration
branch and repeat affected checks. Bring the candidate to Jetson after offline checks;
do not switch branches or pull underneath running control processes.

### When an Upstream Source Change Is Necessary

First consider custom configuration, adapters, or explicit extension interfaces.
When a source change is needed, keep it in a focused feature-branch commit and record
files, rationale, upstream behavior relied on, and regression tests. Reassess it on each
upgrade and remove redundant implementation once upstream incorporates the fix.
General improvements can be prepared separately for contribution upstream.

See [UPSTREAM.md](../software/lerobot-hei-rebot-lift/UPSTREAM.md) for the bundled LeRobot
integration boundary. Normally follow HEI's integrated LeRobot updates; do not separately
merge Hugging Face LeRobot's `main` directly into this repository.

## 8. Validation, Deployment, and Rollback

During independent development, the machines need not stay at identical commits.
**Use the same fork commit/tag for integrated testing and reproducible experiments by
default.** Use different revisions only with recorded protocol compatibility. Matching
upstream versions alone does not establish custom bridge compatibility.

Validate according to impact: links/commands for documentation, unit tests for algorithms,
full actions/units/timeouts/ownership for bridges, and low-speed hardware checks after
offline/simulation checks for hardware changes. Use colcon tests for ROS and relevant
pytest/style checks for LeRobot changes. A documentation edit does not require starting the robot.

Deployment records in `extensions/docs/deployments/` should include:

- Upstream baseline SHA, tested fork candidate SHA, and subsequently created tested tag.
- OS, architecture, JetPack on Jetson, Python/ROS, key dependencies, and manifest revision.
- Configuration/calibration, control mode, model/map/dataset versions.
- Validation steps/results, untested areas, and previous working tag/configuration.

Reference the tested candidate SHA before committing the record, avoiding a self-referential
“final commit SHA” inside its own commit. After validation, commit the record and tag that
commit. Revalidate if code also changed. Adjust the example tag to the actual date:

```bash
git tag -a robot-tested/20261012-01 -m "Validated dual-machine baseline; see deployment record"
git push origin robot-tested/20261012-01
```

Stop relevant processes and ensure clean worktrees on both machines before deploying:

```bash
git fetch origin --tags
git switch --detach robot-tested/20261012-01
# Restore dependencies/configuration from the record; rebuild ROS locally and update Python environments
# Check host/client, feedback, timeouts, and stopping before enabling the task
```

Detached HEAD is suitable for fixed-version operation. Switch back to a feature branch
for development; do not leave field fixes as unbranched commits. Urgent fixes can start
on `fix/*` from the tag and merge back into `dev` after validation.
Rollback likewise requires stopping processes, selecting the previous tested tag, and
restoring matching dependencies, builds, and configuration. **Checking out a commit does
not roll back SDKs, Conda, system packages, or calibration files.** A separate worktree
can isolate a deployment checkout from everyday development, but editable Python installs
and ROS builds still need explicit, correct paths.

## 9. Adoption Sequence

1. Preserve the existing upstream operation path; create minimal `extensions/` structure
   and ignore rules on a feature branch.
2. Check both systems/environments, validate read-only MID360 data and bags, and pin driver/SDK.
3. Implement and test bridge/gateway protocol, full-action holding, unit conversion, and ownership.
4. Add localization, then Nav2; share ROS interfaces and parameter structure across simulation/hardware.
5. Merge each stage into `dev` with repeatable validation. Create a tested tag only when deployment checks pass.

Future modules follow the same directory, dependency, and branch conventions; adding a
sensor does not require redesigning repository management.
