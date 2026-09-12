# S2 — the real-target preflight the nav launch already calls

Status: plan. One PR on top of `rai-nav-hardening`, all under
`experiments/wojtek_rai_v2/`, `ros/src` untouched (`ros/deploy.sh` unaffected).
Verified model-free and against the MuJoCo sim on a laptop, except section 6.

## 1. What the branch does today (file:line on `nav2-stack-plans`)

- `wojtek_rai/nav/launch/nav.launch.py:53` runs `python3 -m
  wojtek_rai.nav.preflight` as an `ExecuteProcess` for `target:=real`
  (`:251-253`). The module does not exist: the process exits non-zero and
  `_after_preflight` (`:255-259`) emits `Shutdown`. Fail-closed by accident;
  the "odometry is live" gate (`:20-22`, `:241-244`) is not implemented.
- Nav2 activation IS gated: `navigation_manager_real` (`:248-250`) is
  returned only from `_after_preflight` on `returncode == 0` and is absent
  from the `LaunchDescription` list (`:265-301`); the sim manager is
  `IfCondition(is_sim)` (`:245-247`). The Nav2 lifecycle nodes start
  unconditionally (`:291-296`) but stay unconfigured, so they publish nothing.
- "Nothing publishes `/cmd_vel` before the preflight" is FALSE. The only
  `/cmd_vel` writer, `watchdog_real` (`:141`, `:284`), starts at launch with
  `stop_burst_on_reconnect:=true` (`:60`) and, before or during the
  preflight, publishes a zero on a `/joint_states` gap
  (`cmd_vel_watchdog.py:138`), bursts 10 zeros when the heartbeat returns
  (`:104-118`), and on the `Shutdown` a failed preflight emits it takes
  SIGINT and sends 5 zeros (`:149-152`): the documented pad stutter (README
  `:209-213`) from a launch that is about to quit.
- The launch comment (`:49-52`) and README (`:194-201`) say the shutdown
  happens "before slam_toolbox, the watchdog or Nav2 start". Wrong:
  `slam_node_real` (`:155-162`), `slam_manager` (`:176-178`), both watchdogs
  and all Nav2 nodes are up before the preflight exits; only activation waits.
- The physical robot publishes a static `odom->base_link` unconditionally
  (`ros/src/wojtek_bringup/wojtek_bringup/launch_common.py:181-188`,
  condition `None` for `hardware == "real"`); the sim skips it for
  `hw:=mujoco` (`:185-187`) and `mujoco_sim_node.py:380-390` broadcasts it
  dynamically. An external odometry node would fight the static one in every
  tf2 buffer, so the preflight must fail when `odom->base_link` is on
  `/tf_static`. Today's robot fails that by construction; lifting it is a
  `ros/` change outside this item.
- `/cmd_vel` is consumed by node `wojtek_policy`
  (`ros/src/wojtek_policy/wojtek_policy/policy_node.py:56`, `:122`), which
  latches it; PR #10 (not in HEAD) adds `cmd_vel_timeout_s`, default 0.0.
- The preflight runs in `wojtek_nav` (`run.sh:173-178`: rclpy, `nav_msgs`,
  `tf2_msgs`, no rai-core, no pytest, `docker/nav.Dockerfile:11-23`); tests
  run in `wojtek_rai:jazzy` (`launch`, rclpy, pytest, `docker/Dockerfile:22-54`).
- `OnProcessExit.handle(event, context)` (launch Jazzy) only extends the
  context locals before the callback, which reads `event.returncode`, so
  `SimpleNamespace(returncode=n)` plus a bare `LaunchContext()` drives it
  model-free. `test_nav_launch.py:43` only walks the top-level `entities`.

## 2. Design

`wojtek_rai/nav/preflight.py`, stdlib + rclpy only (no `rai`, no tf2 buffer:
`/tf` and `/tf_static` are read as `tf2_msgs/TFMessage` to tell them apart).
Read-only by construction: subscriptions and graph queries, never a
publisher; not a tool, so the LLM never reaches it. `rclpy` and the message
imports are function-local in the thin layer (as `topics.py:44` does), so the
module imports on a bare host and section 5.2 runs without ROS. Pure layer:

```python
Sample(recv: float, stamp: float)                       # seconds
Report(now, elapsed, odom: list[Sample], odom_frames, tf_dyn: list[Sample],
       tf_static_seen, cmd_vel_subscribers: tuple[str, ...], heartbeat, depth_publishers)
Thresholds(window_s=5.0, odom_min_hz=10.0, tf_min_hz=10.0, max_age_s=0.5,
           heartbeat_max_age_s=1.0, policy_node="wojtek_policy")
rate_hz(samples, now, window_s); stamps_advance(samples); age_s(samples, now)
judge(report, thr) -> Verdict(ok, reason)
run_until_verdict(collect, now, spin, timeout_s, thr) -> Verdict
```

`judge`, first failure wins, one line each: `no /odom in <t> s`; `/odom
<x> Hz < 10 Hz`; `/odom stamps do not advance`; `/odom stamp <x> s old`;
`/odom frames <a>-><b>, want odom->base_link`; `odom->base_link on
/tf_static (wojtek_bringup placeholder still up)`; `odom->base_link not on
/tf` / `< 10 Hz` / `stamps do not advance`; `/cmd_vel has no subscriber
from wojtek_policy (seen: ...)`; `/joint_states stale <x> s`; `no publisher
on /camera/camera/depth/image_rect_raw`. OK: `preflight OK: /odom <x> Hz
age <y> s, odom->base_link dynamic <z> Hz, /cmd_vel <- wojtek_policy`.

`run_until_verdict` spins in 0.1 s steps, judges every 1 s once `window_s`
has elapsed (rates over the last `window_s`), returns the first OK or, at
`timeout_s` (default 20 s), the last failure. `time.monotonic` for the
deadline, node clock for stamps, no `use_sim_time` (`target:=real` is wall clock, `:267`).

Thin rclpy layer: node `nav_preflight`; `/odom` and `/joint_states` with
sensor-data QoS (matches reliable and best-effort publishers), `/tf` default
QoS, `/tf_static` RELIABLE + TRANSIENT_LOCAL depth 100 (or the latched
placeholder is never seen). `collect()` snapshots the deques plus
`get_subscriptions_info_by_topic("/cmd_vel")` node names and
`count_publishers(DEPTH_IMAGE_TOPIC)`; names from `wojtek_rai/limits.py`
(imports only `os`; `topics.py:15` does the same). `main()`: argparse
`--timeout --window --odom-min-hz`; reason as the last stdout line;
`sys.exit(0 if ok else 1)`; `rclpy.init` failure exits 1.

Launch fix: module-level `gate_actions(returncode, on_ok)` used by the
callback; on success it returns `[watchdog_real, navigation_manager_real]`
and `watchdog_real` leaves the top-level list (`:284`). Sim entities untouched.

## 3. Steps

1. (0.5 h) `tests/test_nav_preflight.py`, pure part, no rclpy import: rate
   over a window; advancing vs frozen stamps; age; each `judge` reason in
   order; `run_until_verdict` on a fake clock: success at 8 s when odometry
   appears at 3 s, failing verdict at 20 s when it never comes, no verdict
   before `window_s`; the module source contains no `create_publisher`.
2. (2.5 h) `wojtek_rai/nav/preflight.py` as in section 2, docstring with the
   checks, exit codes and the `run.sh nav exec` one-liner.
3. (1 h) `nav.launch.py`: `gate_actions`, watchdog gating, rewrite `:49-52`
   (before the gate: slam, unconfigured Nav2; after: watchdog, activation)
   and `:241-244` to list the real checks. `test_nav_launch.py`:
   `_watchdogs_for("real")` also collects from `RegisterEventHandler
   .event_handler.handle(SimpleNamespace(returncode=0), LaunchContext())`;
   add: no top-level `lifecycle_manager_navigation` with `condition None`;
   preflight is `IfCondition(is_real)` with `PREFLIGHT_CMD`; `returncode=1`
   yields `[LogInfo, EmitEvent(Shutdown)]`, no Node; `returncode=0` yields
   one watchdog (burst `true`) then the manager; sim watchdog unchanged.
4. (1 h) README: `target:=real` subsection under Navigation: the command,
   the reasons verbatim, what runs before and after the gate, the robot-side
   prerequisites that do not exist yet (an odometry node for `/odom` +
   dynamic `odom->base_link`; the static placeholder in `wojtek_bringup`
   must go, a `ros/` change; with PR #10, `cmd_vel_timeout_s:=0.5` on the
   robot launch), pad hands-off while the launch is up, arming human-only,
   `/cmd_vel` still in `limits.FORBIDDEN`. Fix Known gaps `:194-201`: the
   preflight exists; today's robot fails it (static TF, no `/odom`).
5. (1.5 h) Section 5 items 4-8, then one PR: `feat(experiments): the
   odometry preflight target:=real has been calling`.

Total about 6.5 h.

## 4. Launch wiring: model-free or not

Importing the launch module covers every entity and condition under a
`LaunchContext` with `target` set, the preflight command and condition, the
real manager's absence from the top level, and the handler's return for any
`returncode`. Not coverable: `OnProcessExit` matching a real `ProcessExited`,
start order and timing, an unconfigured `controller_server` staying silent,
lifecycle transitions, `Shutdown` propagation, DDS/QoS: section 5 (5-8), 6.

## 5. Verification

1. `docker run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 -v "<worktree>/experiments/wojtek_rai_v2:/exp" wojtek_rai:jazzy /entrypoint.sh python3 -m pytest -p pytest_timeout tests -q -p no:cacheprovider`
2. `python3 -m pytest experiments/wojtek_rai_v2/tests/test_nav_preflight.py -q -p no:cacheprovider` on the bare host (no ROS needed).
3. `git -C <worktree> diff --stat origin/rai-nav-hardening -- ros/` prints
   nothing; `git diff --check`.
4. `./experiments/wojtek_rai_v2/run.sh nav build` once (image not built on
   this machine). Sim launch unchanged, on base and PR branch:
   `./experiments/wojtek_rai_v2/run.sh nav exec ros2 launch wojtek_rai/nav/launch/nav.launch.py --print > /tmp/nav_sim_{base,pr}.txt`;
   `diff <(sed -E 's/0x[0-9a-f]+//g' /tmp/nav_sim_base.txt) <(sed -E 's/0x[0-9a-f]+//g' /tmp/nav_sim_pr.txt)`
   is empty. With `target:=real` the only diff is the watchdog line leaving
   the top level.
5. Failure path against the sim (`./ros/sim.sh --foxglove` running: needs
   `wojtek_policy` up, not an armed robot; camera and `/joint_states` are on
   by default, `sim.launch.py:96`, `mujoco_sim_node.py:205`):
   `./experiments/wojtek_rai_v2/run.sh nav exec python3 -m wojtek_rai.nav.preflight; echo exit=$?`
   prints `no /odom in 20.0 s`, `exit=1` (only `/odom` is missing: the sim TF
   is dynamic).
6. Success path: in a second terminal
   `./experiments/wojtek_rai_v2/run.sh nav exec python3 -m wojtek_rai.nav.odom_relay`,
   repeat 5: `preflight OK: ...` within about 8 s, `exit=0`.
7. Static-TF path, relay still up, third terminal:
   `./experiments/wojtek_rai_v2/run.sh nav exec ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link`;
   repeat 5: `odom->base_link on /tf_static ...`, `exit=1`.
8. End to end: `./experiments/wojtek_rai_v2/run.sh nav launch target:=real`
   without the relay shuts down with `nav preflight failed (exit 1)` while
   `ros2 topic hz /cmd_vel` in `wojtek_robot` sees nothing; with the relay
   and a human-armed robot, Nav2 activates after the preflight line and a
   `/navigate_to_pose` goal (plan N1) walks it. Then `run.sh nav launch` (sim).

## 6. Not verifiable without the robot

- The success path on real hardware: no odometry node exists and the static
  `odom->base_link` is up, so the preflight fails at `/tf_static` by design.
- Real QoS of `/odom` (rgbd_odometry or EKF); `/joint_states` rate over the
  AP against the 1.0 s and 10 Hz thresholds; Nav2 activation timing there.

## 7. Stack notes

- S2 goes first: R2 (`nav2-odometry-port.md`) needs the gate to pass on a
  real odometry source, S4 (`nav2-twist-mux.md`) needs the real-target burst
  behind it. S3 (`nav2-panel-estop.md`) shares no code file with S2 and runs
  in parallel; only README Known gaps bullets are adjacent.
- After S2, R2 adds one top-level `leg_odometry_real` entity outside
  `gate_actions` and generalises `_watchdogs_for`; S4 changes `_watchdog()`
  (an extra `-p output_topic`) and adds an unconditioned `twist_mux` Node.
  Both rebase on S2 and keep its handler-walking helper; the tail match of
  `test_nav_launch.py:58` and step 3's "burst `true`" assertion must become
  membership checks once S4 appends its argument.
- The `/cmd_vel <- wojtek_policy` check stays valid after S4 (policy_node
  still subscribes `/cmd_vel`; twist_mux writes it).
- README: S2 and R2 both rewrite `:194-201` (R2's `external_odometry:=true`
  instruction replaces step 4's "prerequisites that do not exist yet");
  PR #10 inserts 7 lines after README `:86`, shifting later references by +7.

## 8. Risks

| risk | handling |
|---|---|
| Passes on stale-but-ticking odometry (stamps advance, pose frozen) | out of scope; N4 acceptance drives a lap; `judge` extensible with a pose-delta check |
| `/tf_static` latched message arrives late | TRANSIENT_LOCAL subscription; no verdict before `window_s` |
| Gating the watchdog delays Nav2's dead-man | same handler return, watchdog first; `/cmd_vel_nav` cannot exist earlier |
