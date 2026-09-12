# Nav2 on Wojtek — the PR stack

Status: 2026-09-13. Two PRs delivered, four planned. Everything here is
laptop-verifiable (model-free tests plus the MuJoCo sim); the physical robot
and the GPU box were not available and every plan says what waits for them.
Parent plans: [rai-on-wojtek.md](rai-on-wojtek.md),
[rai-nav2-on-wojtek.md](rai-nav2-on-wojtek.md) (phase N4 is the robot).

## Delivered

| PR | branch | base | what |
|---|---|---|---|
| #9 | `rai-on-wojtek` | main | RAI agent + Nav2 + perception containers, sim-validated, read-only contact with the robot over the AP (Greg) |
| #11 | `rai-nav-hardening` | #9 | `go_to_object` bounded and cancellable, `stop` cancels the Nav2 goal, `walk` closes on error, stop burst wired for `target:=real`, docs say what actually ran |
| #10 | `policy-cmd-vel-timeout` | #9 | `policy_node` gets `cmd_vel_timeout_s` (default 0.0 = unchanged latch); a nav session starts the robot with 0.5 s |

## Planned, one PR each

| item | plan | base | hours | verifiable here |
|---|---|---|---|---|
| S2 preflight | [nav2-real-preflight.md](nav2-real-preflight.md) | #11 | 6.5 | pure judge + launch wiring by import; sim fail/success paths |
| S3 e-stop | [nav2-panel-estop.md](nav2-panel-estop.md) | #11 | 15 | helper order/timing, tools honour the flag, worker turn; sim drill |
| S4 twist_mux | [nav2-twist-mux.md](nav2-twist-mux.md) | S2 | 10 | config test, teleop parameter tests, launch test; sim walk-through with a pad stand-in |
| R2 odometry | [nav2-odometry-port.md](nav2-odometry-port.md) | S2 | 8.5 | package tests, drift meter against `ros/sim.sh` (accept <= 3 % over 4 m) |

## Order

1. Merge #10 first. It is small, `ros/` only, and all four plans assume
   `cmd_vel_timeout_s:=0.5` in a nav session. Landing it first also removes
   the `cmd_vel_watchdog.py` docstring and `launch_common.py` overlaps before
   anyone rebases.
2. S2 and S3 in parallel on #11. They share no code file, only adjacent
   README bullets. S2 is the gate everything `target:=real` depends on.
3. After S2: R2 and S4 in parallel. Disjoint `ros/` packages
   (`wojtek_odometry` + `launch_common.py` vs `wojtek_teleop` +
   `sim.launch.py`), different regions of `nav.launch.py`, both extend S2's
   launch test helper. The second to land rebases the README Navigation
   section, `rai-nav2-on-wojtek.md` and the `nav.launch.py` docstring. Each
   plan carries a "Stack notes" section with the exact overlaps.
4. Robot phase, human-authorized, separate go: needs S2 + R2 together (the
   preflight passes only with an odometry source and `external_odometry:=true`
   on the robot launch), S3 for the E-STOP drill in the first session, S4
   before the pad is ever used while the nav stack runs.

`wojtek_nav:jazzy` is not built on this machine; `run.sh nav build` once
before any S2/S4/R2 sim check.

## Decisions for the owner

- Whether S4 and R2 may carry their `ros/` parts (teleop `cmd_vel_topic`
  parameter, `sim.launch.py` argument, the `wojtek_odometry` package, the
  `external_odometry` launch argument) inside experiment-stack PRs, or those
  land as separate `ros/` PRs first. Defaults stay unchanged either way, but
  it is deployable code.
- R2 makes leg-kinematic odometry the primary real-robot source and
  `rgbd_odometry` the fallback; the parent plan's N4 says the opposite. Pick
  one for the first robot session.
- S4: route the pad through the laptop-side twist_mux at all? It costs the
  pad its "survives the PC dropping off" property until twist_mux runs on the
  RPi. The alternative is the current rule: pad hands off while the nav
  stack runs. Also pad timeout 1.0 s vs 0.5 s.
- S3: may the panel E-STOP call `/wojtek/arm false` (the click is human, the
  path is WiFi + container + browser), and does "disarm after stop" default
  on. Binding Streamlit to localhost is listed as optional only; the
  network is ours.
- R2: maintainer line for the ported package. The source carries a personal
  address; the repo rule forbids adding new ones.
- S2: preflight thresholds (odom 10 Hz, stamp age 0.5 s, `/joint_states`
  stale 1.0 s, 20 s timeout, 5 s window) are implementer defaults; accept or
  set before the first robot session.
- Who authorizes the N4 acceptance run (preflight on leg odometry, one Nav2
  goal on the robot, the E-STOP drill).

## How each PR is produced

Opus implements from the plan in its own worktree; three review passes
(robot safety, tests and verification honesty, repo rules and hygiene); one
fix round; a sign-off that reruns the suites and opens the draft PR. The
verification commands are in each plan verbatim.
