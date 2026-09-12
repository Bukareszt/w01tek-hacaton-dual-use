# R2 — port wojtek_odometry and feed the nav stack for target:=real

Status: plan. One PR on top of `rai-nav-hardening`: `ros/src/wojtek_odometry`
(ported from `origin/jakuc/odom`), one default-off launch argument in
`wojtek_bringup`, the nav glue under `experiments/wojtek_rai_v2/`. Rebases on
S2 (section 6). `file:line` refer to `nav2-stack-plans`; bare paths are `experiments/wojtek_rai_v2/`.

## 1. What the branch does today

- The physical robot pins `odom->base_link` to identity on `/tf_static`:
  `ros/src/wojtek_bringup/wojtek_bringup/launch_common.py:181-187`, condition
  `None` for `hardware == "real"`; the sim branch skips it for `hw:=mujoco`
  because `mujoco_sim_node` broadcasts the ground truth
  (`ros/src/wojtek_pc/wojtek_pc/mujoco_sim_node.py:210`, `:382-390`) and
  `/odom_vel` (`:209`). No `nav_msgs/Odometry` exists in `ros/`.
- The nav launch relays that ground truth to `/odom` for the sim only
  (`wojtek_rai/nav/launch/nav.launch.py:134-139`, `IfCondition(is_sim)`);
  Nav2 and SLAM read `/odom`, frame `odom` (`wojtek_rai/nav/config/nav2.yaml:11,34,224`,
  `slam_toolbox.yaml:14`). For `target:=real` nothing publishes `/odom`; the
  preflight (`nav.launch.py:53`, `:251-253`; S2 writes it) needs a live
  `/odom` and a dynamic `odom->base_link` before Nav2 activates.
- `origin/jakuc/odom` (merge-base 44b3a76, 102 commits behind this branch) carries
  `ros/src/wojtek_odometry`: `leg_odometry_node` (stance-leg Jacobians + IMU
  orientation, planar pose on `/wojtek/odom`, TF only with `publish_tf`),
  `odom_vs_ground_truth` (sim drift meter), `odom_trace`, `test/test_leg_kinematics.py`.
  128ca69 introduces it; 6e439f1 adds `foot_state()`, `input_stride`, the
  EventsExecutor, one test, and an on-robot placement (bringup exec_depend +
  node + `taskset` budget) that is NOT ported. `config/cyclonedds_lo.xml`
  is 37004cd's loopback DDS profile, unrelated: not ported.
- Every dependency exists here, byte-identical to `jakuc/odom` (`git diff
  origin/jakuc/odom HEAD -- <path>` is empty): `PASSIVE_FROM_KNEE`
  (`ros/src/wojtek_policy/wojtek_policy/poses.py:54-63`, keys
  `<leg>_{fourth,fifth}_joint`), joint names `<leg>_{first,second,third}_joint`
  (`:15-19`), `joint_map.py` + `config/joint_map.yaml`, `wojtek_description/urdf`,
  `wojtek_bringup/urdf/wojtek_real.urdf.xacro`. The foot offset
  `(0.21, 0, 0.0115)` and radius `0.046` in `leg_kinematics.py:31,34` match
  `ros/src/wojtek_description/mujoco/wojtek.xml:71` and `wojtek_mjx.xml:92,95`
  on both branches (`wojtek_mjx.xml` differs elsewhere, not there).
- Inputs on both hardware sides: `/wojtek/joint_states_abs` from
  `real_io_node` per `/joint_states` sample, velocities copied when present
  (`ros/src/wojtek_bringup/wojtek_bringup/real_io_node.py:126`, `:167-174`);
  its passive-joint helper message (`:179-186`) carries no velocity and is
  dropped at `leg_odometry_node.py:206-207`. The manager runs at 200 Hz but
  `joint_state_broadcaster` is paced to 50 Hz (`ros/src/wojtek_bringup/config/real_controllers.yaml:10`,
  `:31-45`, shared by the sim, `launch_common.py:140-149`): about 100
  messages/s on the abs topic, half of them the helper. Velocity/effort exported by
  MD80 (`ros/src/md80_hardware_interface/src/md80_hardware_interface.cpp:83,85`)
  and the MuJoCo plugin (`ros/src/wojtek_mujoco_hardware_interface/src/mujoco_hardware_interface.cpp:246,248,304`).
  IMU `imu_sensor_broadcaster/imu` (`real_controllers.yaml:14-15,47`,
  `mujoco_sim_node.py:208`); `robot_description` latched by rsp (`launch_common.py:171-176`).
- The nav container can run the node: its imports are rclpy, numpy, the msg
  packages, `tf2_ros`, `xml.etree`, `wojtek_policy.poses` (numpy only), and
  `wojtek_nav:jazzy` (`ros-base` + numpy, `docker/nav.Dockerfile:8,22`) runs
  `odom_relay` with `tf2_ros` today; only the two source trees are missing
  (`docker/compose.yaml:68-79`). The control board is a Pi 3
  (`real_controllers.yaml:3-4`); 6e439f1's core budget (policy +
  leg_odometry sharing core 2 at about 65 %) is a Pi 4 number. So the node
  runs on the laptop; on-robot placement waits.

## 2. Design

1. `ros/src/wojtek_odometry`: the package as on `origin/jakuc/odom` minus
   `config/cyclonedds_lo.xml`. Apache-2.0 is already declared in `package.xml`
   and `setup.py`. No `exec_depend` from `wojtek_bringup`: `ros/deploy.sh`
   does not ship it (`--packages-up-to wojtek_bringup`, `ros/deploy.sh:31`);
   `ros/sim.sh` builds it (`ros/sim.sh:211`, all of `src/`).
2. `wojtek_bringup` gets `external_odometry` (default `false`). For
   `hardware == "real"` the static node at `launch_common.py:181-187` gets
   `UnlessCondition(external_odometry)`; the sim branch keeps its `hw`
   condition. (6e439f1 gated it on `use_imu` instead; an explicit argument
   keeps the default robot launch byte-for-byte the same.) Default launch
   unchanged; `external_odometry:=true` only releases the edge, nothing new
   starts on the RPi.
3. Nav glue, real target only: `nav.launch.py` adds top-level
   `leg_odometry_real = ExecuteProcess(cmd=["python3", "-m",
   "wojtek_odometry.leg_odometry_node", "--ros-args", "-p", "publish_tf:=true",
   "-r", "wojtek/odom:=/odom"], cwd="/exp", condition=IfCondition(is_real))`,
   never behind the preflight gate (the preflight waits for `/odom`). Sim
   path (`odom_relay`, `static_map_tf`) untouched. `compose.yaml` mounts
   `../../../ros/src/wojtek_odometry:/wojtek_src/wojtek_odometry:ro` and
   `../../../ros/src/wojtek_policy:/wojtek_src/wojtek_policy:ro` into
   `wojtek_nav` with `PYTHONPATH=/wojtek_src/wojtek_odometry:/wojtek_src/wojtek_policy`
   (`docker/nav-entrypoint.sh:6` prepends `/exp`, keeps the rest). The
   experiment consuming `ros/` is the allowed direction. `/wojtek/odom`
   stays the node's topic (drift meter, `odom_trace`); `/odom` is a remap.

## 3. Steps

1. (1 h) Port: `git -C <worktree> checkout origin/jakuc/odom -- ros/src/wojtek_odometry`,
   delete `config/cyclonedds_lo.xml`. Maintainer line copied from
   `ros/src/wojtek_bringup/package.xml:12` (no new personal address). Fix
   the node docstring's "200 Hz" (`leg_odometry_node.py:13`): the stream is 50 Hz.
   README: replace the `sim.launch.py` steps with section 4.4.
2. (1.5 h) `ros/src/wojtek_odometry/test/test_leg_odometry_node.py`,
   model-free like `tests/test_cmd_vel_watchdog.py`: `_on_efforts`,
   `_on_imu`, `_on_joints_abs` called unbound on a `SimpleNamespace` with
   `LegKinematics` from the xacro URDF and `_publish` stubbed. Four feet
   level, zero rates, zero gyro -> `_vel_base` zero, four in stance; knee
   `|tau|` under `contact_tau_floor` -> none; `dt > 0.1` leaves `_pos`;
   `_quat_to_matrix` identity, `_yaw_of` of a pure yaw.
3. (1 h) `launch_common.py`: `DeclareLaunchArgument("external_odometry",
   default_value="false")` after `use_imu` (`:434`), comment naming who
   owns the edge; condition change at `:181-187`. `docs/ros-architecture.md:159`:
   static TF "unless external_odometry".
4. (2 h) `compose.yaml`; `nav.launch.py` (entity, docstring `:13-22`, comment
   `:131-133`, arg description `:275-276`); `run.sh:59-61` usage.
   `tests/test_nav_launch.py`: `_processes_for(target, module)` generalising
   `_watchdogs_for`; `real` has exactly one `wojtek_odometry.leg_odometry_node`
   with `publish_tf:=true` and `wojtek/odom:=/odom`; `sim` has none and keeps
   `odom_relay`. `tests/test_nav_compose.py`: `wojtek_nav` mounts both trees
   `:ro` and `PYTHONPATH` lists both.
5. (1 h) README: `target:=real` odometry paragraph under Navigation (robot
   launched with `external_odometry:=true`, what the nav container runs,
   `/odom` about 50 Hz, unrun on the robot); fix `:8-10`, `:60-62`, `:194-201`
   (a source exists, unverified there; `WOJTEK_RAI_ODOMETRY=0` stays the
   advice until section 5 is done). `rai-nav2-on-wojtek.md` N4: leg odometry
   first, rgbd_odometry the fallback.
6. (2 h) Section 4, then one PR: `feat(ros): leg odometry for the nav stack
   (wojtek_odometry port)`.

Total about 8.5 h.

## 4. Verification

1. Package tests (the image has `wojtek_policy`, the `wojtek_bringup` share
   and `xacro`; `python3 -m pytest` puts the package dir on `sys.path`):
   `docker run --rm -v "<worktree>/ros/src:/ros2_ws/src" docker-wojtek_robot:latest bash -lc 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash 2>/dev/null; cd /ros2_ws/src/wojtek_odometry && python3 -m pytest test -q -p no:cacheprovider'`
   and the same with `wojtek_bringup`.
2. Experiment suite (`test_repo_hygiene.py` must stay green: no experiment
   name under `ros/`):
   `docker run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 -v "<worktree>/experiments/wojtek_rai_v2:/exp" wojtek_rai:jazzy /entrypoint.sh python3 -m pytest -p pytest_timeout tests -q -p no:cacheprovider`
3. `git diff --check`; inside `./ros/dev.sh`: `ros2 launch wojtek_bringup
   robot.launch.py --show-args` lists `external_odometry` (default false).
4. Drift meter against the laptop sim:
   1. `./ros/sim.sh --foxglove boot_pose:=folded` (builds the package);
      zero -> stand_up -> arm from http://localhost:8080.
   2. `docker exec -it wojtek_robot bash -lc 'source /ros2_ws/install/setup.bash && ros2 run wojtek_odometry leg_odometry_node'`
      (`publish_tf` false: the plant owns the TF in the sim).
   3. `docker exec -it wojtek_robot bash -lc 'source /ros2_ws/install/setup.bash && ros2 run wojtek_odometry odom_vs_ground_truth'`
   4. 15 s straight, then a zero (the policy latches the last command):
      `docker exec -it wojtek_robot bash -lc 'source /ros2_ws/install/setup.bash && ros2 topic pub -r 20 -t 300 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" && ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"'`;
      then 20 s of `{angular: {z: 0.5}}` the same way.
   5. Accept `drift` at or below 3 % over at least 4 m, `yaw_err` under 2 deg.
      The reference (1.1 % straight, 2.7 % arc) is from `jakuc/odom`'s sim and
      policy: report the number; retune `rolling_radius` only above 5 %.
5. Nav-container path, sim standing in for the robot:
   `./experiments/wojtek_rai_v2/run.sh nav exec python3 -m wojtek_odometry.leg_odometry_node --ros-args -p publish_tf:=false -r wojtek/odom:=/odom`;
   in `wojtek_robot`, `ros2 topic hz /odom` about 50 Hz and `ros2 topic echo
   /odom --once` shows `odom` / `base_link`. Proves imports and mounts.
6. `./experiments/wojtek_rai_v2/run.sh nav launch` (sim) unchanged: `--print`
   diff against the base as in `nav2-real-preflight.md` section 5.4. `target:=real`
   against the sim only smoke-tests the gate (two `odom->base_link` writers).

## 5. Not verifiable without the robot

- `/wojtek/joint_states_abs` at 50 Hz (two messages per sample) and the IMU
  over the AP: loss, jitter, the `dt > 0.1` drop (`leg_odometry_node.py:218`);
  whether `input_stride` must rise on the laptop.
- Real drift: ESKF yaw on top of the sim numbers, carpet slip,
  `contact_tau_floor` 0.8 N*m and `contact_z_delta` 3 mm on real knees.
- `external_odometry:=true` on the RPi service; S2's preflight passing on
  this source; a Nav2 goal on leg odometry with scan matching; all of N4
  (human-authorized, pad hands off).

## 6. Stack notes

- Order: S2 first, R2 rebased on it. S2 edits `nav.launch.py` (gate,
  watchdog, `:49-52`, `:241-244`), `test_nav_launch.py`, README Known gaps.
  R2 adds one top-level entity and edits `:13-22`, `:131-133`, README
  `:194-201`: odometry stays top-level, S2's `gate_actions` untouched, step
  4's `_processes_for` generalises S2's handler-walking helper, README
  paragraphs merged, S2's "prerequisites that do not exist yet" replaced by
  the `external_odometry:=true` instruction.
- S4 (`nav2-twist-mux.md`) runs in parallel: disjoint `ros/` packages,
  adjacent `nav.launch.py` docstring lines (`:9-11` vs `:13-22`), both edit
  README Navigation and `rai-nav2-on-wojtek.md` (S4 `:199`, R2 N4); second
  to land rebases. S3 overlaps nothing.
- PR #10 edits `launch_common.py:220-230` and inserts after `:435`; R2
  inserts after `:434`: trivial textual conflict, keep both. PR #10 also
  inserts 7 lines after README `:86` (later references shift by +7).

## 7. Risks

| risk | handling |
|---|---|
| Two `odom->base_link` writers on the robot (static left on) | `external_odometry` must be set; S2's `/tf_static` check refuses Nav2 otherwise |
| Odometry over WiFi stalls: pose freezes while Nav2 drives | watchdog dead-man and S2's stamp checks; a `/joint_states` gap also stops the watchdog |
| Drift numbers do not reproduce on this branch's sim/policy | report, keep constants; a `rolling_radius` refit is its own PR with the meter log |
| `docker-wojtek_robot:latest` predates the port | `ros/sim.sh` rebuilds src; the test command needs only the source dir |
| Laptop placement vs later on-robot placement | the node is placement-agnostic; on-robot needs 6e439f1's measured budget, not this PR |
| `WOJTEK_RAI_ODOMETRY` now depends on what runs | README says when to set it; the agent still never reaches `/cmd_vel` or the arm services |
