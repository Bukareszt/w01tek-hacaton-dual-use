# S4 — twist_mux on `/cmd_vel`, the pad on top

Status: plan, not started. One PR on top of PR #11 (rai-nav-hardening),
touching `experiments/wojtek_rai_v2/`, `ros/src/wojtek_teleop/` and one
launch argument in `ros/src/wojtek_pc/launch/sim.launch.py`. `file:line`
refer to branch `nav2-stack-plans`; paths without a directory are
`experiments/wojtek_rai_v2/`. S2 (`nav2-real-preflight.md`) and S3
(`nav2-panel-estop.md`) are siblings; this item only decides who may write `/cmd_vel`.

twist_mux, in two sentences: a stock ROS 2 node (`ros-jazzy-twist-mux`, 4.5.1
on Jazzy) subscribes to several `Twist` topics, each with a priority and a
timeout, and republishes on one output topic only the messages of the
highest-priority input that has published within its timeout. It forwards and
never invents messages: a quiet winner's last message stays where it landed,
and after its timeout the next input down takes over.

## 1. Every `/cmd_vel` writer on the robot path today

`policy_node` is the one reader (`ros/src/wojtek_policy/wojtek_policy/policy_node.py:122`),
latching by default; PR #10 (not in HEAD) adds `cmd_vel_timeout_s`, 0 by
default, 0.5 for a nav session. Nothing arbitrates; the last message wins.

| writer | where it runs, launched by | rate | goes quiet? | muxed by this PR |
|---|---|---|---|---|
| `gamepad_teleop` | RPi service (`ros/deploy/wojtek-robot.service:72` `gamepad:=true`) and the sim (`sim.launch.py:88`); include `launch_common.py:535-544` | 20 Hz while LIVE/ZEROING (`gamepad_teleop.py:72,130,282-307`) | 0.5 s dead-man, then 2 s of zeros, then silent (`pad_drive.py:79-84`, `gamepad_teleop.py:96`) | yes, priority 100 |
| `text_commander` | sim always (`sim.launch.py:137-141`); on the robot by hand (`ros2 run`, README.md:5-8) | 20 Hz while a command is active (`text_commander.py:27,88,104-111`) | 2 s dead-man, exactly one zero, then silent (`:69-76`) | yes, priority 50 |
| Nav2 through `cmd_vel_watchdog` | `wojtek_nav` container, `nav.launch.py:140-141,283-284` | velocity_smoother 20 Hz (`nav/config/nav2.yaml:212`) on `/cmd_vel_nav` (`nav.launch.py:238`), relayed (`cmd_vel_watchdog.py:92-95`) | one zero 0.5 s after Nav2 stops (`:120-126`); real target: 10 zeros after a heartbeat gap (`:104-118`); 5 zeros on SIGINT (`:86-90,149-152`) | yes, priority 10, on a nav-only topic |
| `deck_gateway` | RPi service `deck:=true` (`launch_common.py:297-321`), sim default (`:575-577`) | 20 Hz (`deck_gateway.py:103,176,395-400`) | 0.5 s, 2 s zeros, silent (`:105-106`, `drive.py:12-15`) | no — known gap |
| `web_console` / `console` (Qt) | PC only (`sim.launch.py:148-166`, `robot.py:229-235`) | 20 Hz while driving (`web_console.py:86-88,112`, `operator_console.py:75,108`) | 0.5 s (web) | no — known gap |
| Foxglove Teleop panel, `teleop_twist_keyboard` | PC, by hand (`viz.launch.py:9-10,33`, `sim.launch.py:39`) | panel's own rate / one message per keypress | never; rely on the latch (PR #10 comment) | no — known gap |

Per-input timeouts follow from the table: every muxed source streams at 20 Hz
and zeroes before it goes quiet, so a 0.5 s gap means the source is gone,
not idle. The pad gets 1.0 s: it streams over WiFi, and a hiccup shorter than
its timeout must not let a Nav2 message through (the mux only decides who may
speak; a longer pad timeout delays lower inputs and holds nothing stale). The
mux adds no messages, so with PR #10 the freshness `policy_node`'s 0.5 s
dead-man sees is the winning input's, exactly as without the mux.

## 2. Design

- `docker/nav.Dockerfile:11-23`: add `ros-${ROS_DISTRO}-twist-mux`.
- `wojtek_rai/nav/config/twist_mux.yaml`:
  ```yaml
  twist_mux:
    ros__parameters:
      use_stamped: false     # 4.5.1 defaults to true = TwistStamped; policy_node reads Twist
      topics:
        pad:  {topic: /cmd_vel_pad,  timeout: 1.0, priority: 100}
        text: {topic: /cmd_vel_text, timeout: 0.5, priority: 50}
        nav:  {topic: /nav/cmd_vel,  timeout: 0.5, priority: 10}
  ```
  No `locks` group (4.5.x enumerates prefixes; absent = empty; checked in section 5).
- `nav.launch.py`: module constants `CMD_VEL_TOPIC = "/cmd_vel"`,
  `NAV_GUARDED_TOPIC = "/nav/cmd_vel"` (same `/nav/` prefix as `:116-118,127`);
  a `Node(package="twist_mux", executable="twist_mux", name="twist_mux",
  parameters=[twist_mux.yaml, common], remappings=[("cmd_vel_out",
  CMD_VEL_TOPIC)])` with no condition (both targets); `_watchdog()` (`:63-69`)
  gains `-p output_topic:=/nav/cmd_vel`; docstring `:9-11` and the comment
  `:200-201` say the mux is the only writer of `/cmd_vel`.
- `cmd_vel_watchdog.py`: default `output_topic` becomes `/nav/cmd_vel`
  (`:59`) so a bare `ros2 run` cannot bypass the mux either; docstring `:3-4`
  and `:23-29` rewritten (the burst now lands on the nav input and can no
  longer interleave with the pad; the pad wins by priority).
- `wojtek_rai/limits.py:76-84`: `FORBIDDEN` gains `/cmd_vel_pad`,
  `/cmd_vel_text`, `/nav/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_raw`; the tool
  list (`tools.py:62-65`) does not change. The LLM still writes only
  `/wojtek/nav_command`.
- `ros/src/wojtek_teleop`: `cmd_vel_topic` parameter, default `"cmd_vel"`, in
  `text_commander.py:86` and `gamepad_teleop.py:128` (`declare_parameter`
  next to the others, `:82-99`); the `info` line logs it. Both `__init__`
  take `**node_kwargs` passed to `Node.__init__` so a test can hand in
  `parameter_overrides` (only `main()` constructs them, `:114` / `:310`).
  `gamepad.launch.py`: `DeclareLaunchArgument("gamepad_cmd_vel_topic",
  default_value="cmd_vel")` passed as the parameter (`:60`); a value given to
  `robot.launch.py` / `sim.launch.py` reaches the include the way `policy`
  does today (`launch_common.py:535-544` passes no `launch_arguments`).
  `sim.launch.py`: `text_cmd_vel_topic` (default `cmd_vel`) on the
  `text_commander` node (`:137-141`). Defaults unchanged, so `ros/deploy.sh`
  and the service (`wojtek-robot.service:72`) behave as before.
- A nav session passes `gamepad_cmd_vel_topic:=/cmd_vel_pad
  text_cmd_vel_topic:=/cmd_vel_text` (plus `cmd_vel_timeout_s:=0.5` once
  PR #10 is in) to `ros/sim.sh` (`name:=value` pass through, `ros/sim.sh:10-12`, `:70-83`);
  on the robot a human starts `robot.launch.py` with the pad argument, and
  `text_commander` by hand with `-p cmd_vel_topic:=/cmd_vel_text`.
- Not muxed, README known gaps: deck gateway, both PC consoles, Foxglove
  panel, keyboard keep writing `/cmd_vel` unarbitrated; "one drive surface at
  a time" stays the rule. Not doing: twist_mux on the RPi (package in
  `ros/deploy/rpi/install.sh` + `robot.launch.py`, a later `ros/` item),
  `locks`, a "pad active" lock, changes to `wojtek_deck`/`wojtek_pc` nodes or Nav2 params.

## 3. Steps (about 10 h)

| # | step | h |
|---|---|---|
| 1 | `nav.Dockerfile`, `twist_mux.yaml`, `nav.launch.py` (constants, node, watchdog argument, docstrings) | 2 |
| 2 | `cmd_vel_watchdog.py` default + docstring; `limits.FORBIDDEN` | 0.5 |
| 3 | `tests/test_twist_mux_config.py`, `test_nav_launch.py`, `test_limits.py` (section 4) | 2 |
| 4 | `wojtek_teleop`: parameter in both nodes, `**node_kwargs`, `gamepad.launch.py` argument; `sim.launch.py` argument | 1.5 |
| 5 | `test_gamepad_teleop_node.py` + new `test_text_commander_node.py` (section 4) | 1 |
| 6 | Docs: README diagram `:111` and Known gaps `:209-213` (replace with the gap list of section 2), nav session command `:114-118`; `docs/ros-architecture.md:262-300` parameter lines; `sim.launch.py:36-44` docstring; `launch_common.py:526-527` comment; `rai-nav2-on-wojtek.md:199` handling column | 1.5 |
| 7 | Section 5, fix-ups, one PR: `feat: twist_mux arbitrates /cmd_vel, the pad on top` | 1.5 |

## 4. Tests

- `tests/test_twist_mux_config.py` (model-free, `yaml` only): key
  `twist_mux/ros__parameters`; `use_stamped` false; exactly `pad`, `text`,
  `nav`; priorities strictly `pad > text > nav`; timeouts in `(0, 1.0]`, `nav`
  and `text` = 0.5 (the watchdog's `timeout_s`, `:60`); topics distinct, none
  `/cmd_vel`, all in `limits.FORBIDDEN`; `nav` = `nav.launch.NAV_GUARDED_TOPIC`.
- `test_nav_launch.py`: the watchdog command (`_plain_cmd`, `:28-36`) carries
  `-p output_topic:=/nav/cmd_vel` for both targets; exactly one `twist_mux`
  Node, no condition, parameters listing `twist_mux.yaml`, remap
  `cmd_vel_out -> /cmd_vel` (`node._perform_substitutions(context)` with
  `target` and `params_file` set, then `node.expanded_remapping_rules`; if
  that needs the file, `params_file` is the repo config, present on the
  host); no other entity remaps to `/cmd_vel`; docstring `:62-64` now says
  the mux is the only writer. Keep S2's helper if it landed first.
- `test_limits.py:40-48`: the five new names in `FORBIDDEN`, still disjoint
  from `WRITABLE_*`. `test_cmd_vel_watchdog.py` (stub node, `:42-58`) gains
  one check that the source's `output_topic` default is `/nav/cmd_vel`.
- `ros/src/wojtek_teleop/test/test_gamepad_teleop_node.py` (rclpy, skipped on
  the host, `:19`): `GamepadTeleop()` has `_pub_cmd.topic_name == "/cmd_vel"`;
  `GamepadTeleop(parameter_overrides=[Parameter("cmd_vel_topic", STRING,
  "/cmd_vel_pad")])` publishes on `/cmd_vel_pad`. New
  `test_text_commander_node.py`, same skeleton (`:9-35`), same two checks;
  `test_text_commander.py` (pure `CommandState`) untouched.

## 5. Verification

```bash
W=<worktree>
./experiments/wojtek_rai_v2/run.sh nav build         # wojtek_nav:jazzy is not built on this machine
# the two facts the config relies on: version 4.5.1, use_stamped False, one Twist publisher, no "locks" error
./experiments/wojtek_rai_v2/run.sh nav exec bash -lc 'ros2 pkg xml twist_mux | grep -m1 "<version>"; ros2 run twist_mux twist_mux --ros-args --params-file wojtek_rai/nav/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel & sleep 3; ros2 param get /twist_mux use_stamped; ros2 topic info /cmd_vel; kill $!'
docker run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 -v "$W/experiments/wojtek_rai_v2:/exp" wojtek_rai:jazzy /entrypoint.sh python3 -m pytest -p pytest_timeout tests -q -p no:cacheprovider
docker run --rm -v "$W/ros/src:/ros2_ws/src" docker-wojtek_robot:latest bash -lc 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash 2>/dev/null; cd /ros2_ws/src/wojtek_teleop && python3 -m pytest test -q -p no:cacheprovider'
git -C "$W" diff --check
git -C "$W" diff --stat origin/rai-nav-hardening -- ros/   # only wojtek_teleop/* and wojtek_pc/launch/sim.launch.py
./experiments/wojtek_rai_v2/run.sh nav exec ros2 launch wojtek_rai/nav/launch/nav.launch.py --print   # new: twist_mux, watchdog output_topic; nothing else
```
Sim on the laptop (README "Run it"; `cmd_vel_timeout_s` only with PR #10):
```bash
./ros/sim.sh --foxglove boot_pose:=folded gamepad_cmd_vel_topic:=/cmd_vel_pad text_cmd_vel_topic:=/cmd_vel_text cmd_vel_timeout_s:=0.5
./experiments/wojtek_rai_v2/run.sh nav launch
./experiments/wojtek_rai_v2/run.sh nav exec ros2 topic info /cmd_vel -v      # one publisher: twist_mux
./experiments/wojtek_rai_v2/run.sh nav exec ros2 topic echo /cmd_vel --field linear.x
```
(a) `run.sh chat "go to the hydrant"`: `/cmd_vel` follows `/nav/cmd_vel`.
(b) During the goal, `run.sh nav exec ros2 topic pub -r 20 /cmd_vel_pad
geometry_msgs/msg/Twist "{linear: {x: 0.0}}"` (pad stand-in; the macOS
container sees no pad, `ros/sim.sh:101`): `/cmd_vel` reads 0 within 0.1 s
while Nav2 keeps streaming; Ctrl-C the pub: Nav2 is back about 1 s later
(section 7, first row). (c) `run.sh chat "walk forward for 3 seconds"`:
`/cmd_vel_text` carries 0.3 for 3 s, one zero, then `/cmd_vel` is silent
until Nav2 speaks. (d) Ctrl-C the nav launch: the watchdog's 5 zeros reach
`/cmd_vel`. (e) `./ros/sim.sh --foxglove boot_pose:=folded` without the
arguments, no nav container: `/cmd_vel` written directly, today's behaviour.

## 6. Not verifiable without the robot

- The pad through the laptop: `/cmd_vel_pad` crosses the AP to `wojtek_nav`,
  `/cmd_vel` crosses back; latency and loss unmeasured, stacked with the pad's
  0.5 s dead-man (`gamepad_teleop.py:96`) and the 1.0 s mux timeout.
- The real-target burst (`nav.launch.py:60`) losing to a driving pad:
  `target:=real` has never run (README.md:194-201) and needs S2 first.
- `text_commander` on the robot with `-p cmd_vel_topic:=/cmd_vel_text`
  (discovery delay, `limits.py:101-104`); the manual `robot.launch.py` start
  with the pad argument, a human N4 step (`rai-nav2-on-wojtek.md:176-178`).

## 7. Stack notes

- Rebase on S2 (`nav2-real-preflight.md`): after it `watchdog_real` is
  returned by `gate_actions`, so the `_watchdog()` change reaches both
  targets untouched and the twist_mux Node stays top-level (no condition,
  never inside the gate). Section 4's watchdog test uses S2's handler-walking
  helper; S2's tail match on the command (`test_nav_launch.py:58`) becomes
  a membership check because this PR appends `-p output_topic:=...`.
- R2 (`nav2-odometry-port.md`) runs in parallel: disjoint `ros/` packages
  (`wojtek_teleop` + `sim.launch.py` here, `wojtek_odometry` +
  `launch_common.py` there), adjacent regions of the `nav.launch.py`
  docstring (`:9-11` here, `:13-22` there), both edit README Navigation and
  `rai-nav2-on-wojtek.md` (`:199` here, N4 there): second to land rebases.
- PR #10 rewrites `cmd_vel_watchdog.py:2-10` and `:20-28`, the same
  docstring step 2 rewrites, and inserts 7 lines after README `:86`
  (later README references shift by +7): re-do the docstring on whichever
  lands second. S3 overlaps only in README Known gaps (`:202-208` vs
  `:209-213`).

## 8. Risks

| risk | handling |
|---|---|
| Pad release hands `/cmd_vel` back to a Nav2 goal still in flight (mux timeouts, no cancel) | documented rule: the pad overrides, it does not cancel; cancel from the panel (S3) or disarm; seen in sim step (b) |
| With the mux on the laptop, the pad no longer "survives the PC dropping off" (`robot.py:240-243`): a dead link leaves the pad mute | only in a nav session, by explicit argument; the A-button disarm is a local RPi service (`gamepad_teleop.py:114,232-258`) and keeps working; PR #10's dead-man stands the robot; twist_mux on the RPi is the later fix |
| twist_mux 4.5.1 publishes `TwistStamped` by default | `use_stamped: false` in the YAML, asserted by the config test and step 5's `ros2 param get` |
| An empty `locks` group refuses to start | step 5 runs the node standalone first; fallback: one lock on `/nav/cmd_vel_lock`, timeout 0, documented |
| A source not remapped keeps writing `/cmd_vel` around the mux (deck, consoles, keyboard) | listed as gaps; `ros2 topic info /cmd_vel -v` shows exactly one publisher in the sim check |
| Experiment hygiene test scans `ros/` for the experiment's name (`test_repo_hygiene.py:54-70`) | the `ros/` docstrings and comments name "the nav stack", never `wojtek_rai_v2` |
