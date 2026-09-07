# Two-Machine Verification (FOUND-07)

This document proves that `run.sh install`, `run.sh test`, and the
topic-discovery target (`run.sh agent-topics`) succeed on both an x86-64
machine and the aarch64 remote GPU dev box, with literal command output
recorded from each. It discharges requirement FOUND-07.

**Commit under verification:** `3693ae69a4c95a1442f37edc589916e32d4e1076`

**Redaction note:** the second machine is identified only by role — "the
aarch64 remote GPU dev box" — never by hostname, address, login, or SSH
alias. Any pasted output line that would otherwise reveal a host name,
address, login, or a filesystem path identifying private infrastructure is
redacted before being committed here, and every redaction is marked inline
as `[REDACTED: <what>]`.

---

## x86-64 laptop

- **Architecture (`uname -m`):** `x86_64`
- **Container image identity (as reported by the container):**
  - Compose service image tag: `docker-wojtek_robot`
  - Image ID: `sha256:0c6abf45215fe168542e6e0b25ffe16727a4bb55547b1956f618b932a7a97c77`
  - No registry digest — image was built locally by `docker compose`, not
    pulled from a registry.
- **Commit SHA the run was made from:** `3693ae69a4c95a1442f37edc589916e32d4e1076`
  (repository `git status --porcelain` was empty immediately before this run,
  per this plan's Task 1 precondition).

### 1. `./experiments/wojtek_rai_v1/run.sh install`

Second and later runs are idempotent (guarded by `dpkg -s` / `[ -x ... ]` /
`[ -f uv.lock ]` checks in `run.sh`); this is the literal tail of a real run
against the already-provisioned container. The `apt-get`/`dpkg` package
listing in the middle (over a thousand lines resolving and unpacking
`ros-jazzy-cv-bridge`'s dependency tree) is truncated — marked explicitly
below — since it is routine package-manager noise, not something this
document needs verbatim.

```
 Container wojtek_robot  Running
>> installing ros-jazzy-cv-bridge into the running container
Get:1 http://archive.ubuntu.com/ubuntu noble InRelease [256 kB]
Get:2 http://security.ubuntu.com/ubuntu noble-security InRelease [126 kB]
...
[TRUNCATED: ~1040 lines of apt-get/dpkg package resolution and unpacking
output for ros-jazzy-cv-bridge and its dependency tree]
...
Setting up ros-jazzy-cv-bridge (4.1.0-1noble.20260615.144656) ...
Processing triggers for libc-bin (2.39-0ubuntu8.7) ...
   Building wojtek-rai-v1 @ file:///ros2_ws/experiments/wojtek_rai_v1
      Built wojtek-rai-v1 @ file:///ros2_ws/experiments/wojtek_rai_v1
Prepared 1 package in 775ms
Uninstalled 1 package in 1ms
Installed 1 package in 1ms
 ~ wojtek-rai-v1==0.1.0 (from file:///ros2_ws/experiments/wojtek_rai_v1)
>> install complete: /ros2_ws/experiments/wojtek_rai_v1/.venv
```

**Exit status:** `0`

### 2. `./experiments/wojtek_rai_v1/run.sh test`

```
 Container wojtek_robot  Running
.................................................................        [100%]
=============================== warnings summary ===============================
tests/test_topics_module.py::test_main_is_callable_with_optional_argv
  /ros2_ws/experiments/wojtek_rai_v1/.venv/lib/python3.12/site-packages/pydub/utils.py:14: DeprecationWarning: 'audioop' is deprecated and slated for removal in Python 3.13
    import audioop

tests/test_topics_module.py::test_main_is_callable_with_optional_argv
  /ros2_ws/experiments/wojtek_rai_v1/.venv/lib/python3.12/site-packages/pydub/utils.py:170: RuntimeWarning: Couldn't find ffmpeg or avconv - defaulting to ffmpeg, but may not work
    warn("Couldn't find ffmpeg or avconv - defaulting to ffmpeg, but may not work", RuntimeWarning)

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
65 passed, 2 warnings in 45.95s
```

**Exit status:** `0`

### 3. `./experiments/wojtek_rai_v1/run.sh agent-topics`

The simulation was already running in the `wojtek_robot` container's ROS 2
graph before this command was run — started with the same
`ros2 run wojtek_bringup robot --sim ...` invocation `run.sh up` execs
(the container lifecycle decision recorded at the plan 01-01 checkpoint),
so that live `cmd_vel` and camera topics existed for `agent-topics` to
discover through RAI's own `ROS2Connector`.

```
 Container wojtek_robot  Running
/ros2_ws/experiments/wojtek_rai_v1/.venv/lib/python3.12/site-packages/pydub/utils.py:170: RuntimeWarning: Couldn't find ffmpeg or avconv - defaulting to ffmpeg, but may not work
  warn("Couldn't find ffmpeg or avconv - defaulting to ffmpeg, but may not work", RuntimeWarning)
2026-09-07 14:51:20 [REDACTED: hostname] ROS2Connector[3510] WARNING Auto-initializing ROS2, but manual initialization is recommended. For better control and predictability, call rclpy.init() or ROS2Context before creating this connector.
/camera/camera/color/camera_info  ['sensor_msgs/msg/CameraInfo']
/camera/camera/color/image_raw  ['sensor_msgs/msg/Image']
/camera/camera/depth/camera_info  ['sensor_msgs/msg/CameraInfo']
/camera/camera/depth/image_rect_raw  ['sensor_msgs/msg/Image']
/cmd_vel  ['geometry_msgs/msg/Twist']
/controller_manager/activity  ['controller_manager_msgs/msg/ControllerManagerActivity']
/controller_manager/introspection_data/full  ['pal_statistics_msgs/msg/Statistics']
/controller_manager/introspection_data/names  ['pal_statistics_msgs/msg/StatisticsNames']
/controller_manager/introspection_data/values  ['pal_statistics_msgs/msg/StatisticsValues']
/controller_manager/statistics/full  ['pal_statistics_msgs/msg/Statistics']
/controller_manager/statistics/names  ['pal_statistics_msgs/msg/StatisticsNames']
/controller_manager/statistics/values  ['pal_statistics_msgs/msg/StatisticsValues']
/diagnostics  ['diagnostic_msgs/msg/DiagnosticArray']
/dynamic_joint_states  ['control_msgs/msg/DynamicJointState']
/forward_effort_controller/commands  ['std_msgs/msg/Float64MultiArray']
/forward_effort_controller/transition_event  ['lifecycle_msgs/msg/TransitionEvent']
/forward_position_controller/commands  ['std_msgs/msg/Float64MultiArray']
/forward_position_controller/transition_event  ['lifecycle_msgs/msg/TransitionEvent']
/imu_sensor_broadcaster/imu  ['sensor_msgs/msg/Imu']
/imu_sensor_broadcaster/transition_event  ['lifecycle_msgs/msg/TransitionEvent']
/joint_state_broadcaster/transition_event  ['lifecycle_msgs/msg/TransitionEvent']
/joint_states  ['sensor_msgs/msg/JointState']
/magnetometer_broadcaster/magnetic_field  ['sensor_msgs/msg/MagneticField']
/magnetometer_broadcaster/transition_event  ['lifecycle_msgs/msg/TransitionEvent']
/odom_vel  ['geometry_msgs/msg/Twist']
/parameter_events  ['rcl_interfaces/msg/ParameterEvent']
/robot_description  ['std_msgs/msg/String']
/rosout  ['rcl_interfaces/msg/Log']
/sim/qpos  ['std_msgs/msg/Float64MultiArray']
/sim/rtf  ['std_msgs/msg/Float32']
/tf  ['tf2_msgs/msg/TFMessage']
/tf_static  ['tf2_msgs/msg/TFMessage']
/wojtek/joint_states_abs  ['sensor_msgs/msg/JointState']
/wojtek/joint_targets  ['sensor_msgs/msg/JointState']
/wojtek/nav_command  ['std_msgs/msg/String']
```

`[REDACTED: hostname]` stands for this development machine's own hostname,
which `rclpy`'s default log formatter embeds in every log line
(`ROS2Connector`'s `WARNING` line above) regardless of container boundary —
it is not part of the ROS 2 graph output the criterion cares about, and is
redacted per this document's own redaction rule rather than left in because
it happens to originate on the "known" machine.

`/cmd_vel` and both camera streams (`color` and `depth`) are present,
satisfying the plan's backstop truth.

**Exit status:** `0`

---

## aarch64 remote GPU dev box

_Pending — see Task 2 of this plan. The dev-box section is appended once the
operator supplies the captured output from that machine._

---

## Findings

_Findings from either machine's run — differences in resolved dependency
versions, container start-up behavior, or architecture-specific issues —
land here. Empty until the aarch64 section above is filled in._
