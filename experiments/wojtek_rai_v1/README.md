# Experiment: RAI (RobotecAI) integration for Wojtek, v1

## Status

> **EXPERIMENTAL. Not production, not on the robot.**
> Nothing here is deployed by `ros/deploy.sh`, and no package here is a
> dependency of `wojtek_bringup`. Treat every interface in this directory as
> unstable: names, topics and message fields change without a migration
> path.

## Scope

At the end of Phase 1, this experiment is a pinned RAI (RobotecAI) Python
environment that runs inside Wojtek's existing `wojtek_robot` dev container,
joins the running MuJoCo simulation's ROS 2 graph over the same DDS domain
the robot already uses, and lists that graph's topics -- including
`cmd_vel` and the D435 camera streams -- through RAI's own
`ROS2Connector`, not a hand-rolled `rclpy` call.

It is deliberately **not yet** an agent: no ReAct loop, no tool registry, no
chat interface, no LLM call. `config.toml` ships RobotecAI's own upstream
model-vendor defaults, unused. Phases 2 through 5 in
`.planning/ROADMAP.md` add, in order, a chatting agent, a velocity arbiter
that lets a typed instruction move the simulated robot, vision tools, and
navigation -- each on top of this same pinned environment, without
revisiting the decisions recorded here.

## Usage

Every `run.sh` subcommand below brings up the `wojtek_robot` container
itself, idempotently, before doing anything else -- there is no separate
"start the container first" step to remember (the Task 1 checkpoint
decision, plan `01-01`: `run.sh` owns the container's lifecycle rather than
depending on `ros/sim.sh` or `ros/dev.sh` to have started it with this
experiment's compose override already attached, since neither of those
scripts has an extension point for an extra compose file). Run them from
the repository root, in this order for a first-time setup:

```
./experiments/wojtek_rai_v1/run.sh install       pinned venv inside the wojtek_robot container (uv + uv.lock)
./experiments/wojtek_rai_v1/run.sh build         vcs import + rosdep + colcon build of rai_interfaces into ros_ws/
./experiments/wojtek_rai_v1/run.sh test          model-free unit tests -- no ROS runtime, no LLM key, no GPU
./experiments/wojtek_rai_v1/run.sh container     bring up wojtek_robot with this experiment's bind mount, idempotently
./experiments/wojtek_rai_v1/run.sh agent-topics  print the running simulation's live ROS 2 topic list
./experiments/wojtek_rai_v1/run.sh up            start the sim (ros/sim.sh's session) with this experiment mounted
./experiments/wojtek_rai_v1/run.sh agent         run the RAI agent process (not implemented until Phase 2)
```

`install` and `build` are one-time setup (safe to re-run; both are
idempotent). `test` needs no simulation running -- it is model-free by
design (FOUND-06). `agent-topics` needs a simulation actually running
somewhere on the same DDS domain to see any topics beyond its own; start
one first with `up` (in another terminal) or with `ros/sim.sh` directly.
`container` is rarely needed by name -- every other subcommand calls it
internally -- but is useful on its own to just bring the stack up without
running anything against it. `agent` is a stub until Phase 2.

## Layout

| path | purpose |
|---|---|
| `run.sh` | entry point: `install \| build \| test \| container \| agent \| agent-topics \| up` |
| `pyproject.toml`, `uv.lock` | pinned `rai-core`/`rai-whoami` direct deps; `uv.lock` is the only pin on RAI's unconstrained LangChain/LangGraph transitives (D-07, D-08) |
| `config.toml` | RAI's upstream vendor/tracing config template, committed verbatim -- no credential field exists in its schema at all (D-12) |
| `docker/compose.override.yaml` | extends `wojtek_robot` with this experiment's bind mount, never edits `ros/docker/compose.yaml` (D-02, D-04) |
| `ros/rai_interfaces.repos` | `vcs` pin for `rai_interfaces` at a commit SHA, never a branch or tag (D-09) |
| `ros_ws/` | the colcon overlay `run.sh build` produces, outside `ros/src/` -- gitignored, rebuilt per machine |
| `wojtek_rai/` | this experiment's own Python package: `topics.py` (ROS 2 topic discovery), `config.py` (cwd-independent `config.toml` loader) |
| `tests/` | model-free guard and unit tests -- no ROS runtime, no LLM key, no GPU (FOUND-06) |
| `.venv/`, `.tools/`, `.uv-cache/` | gitignored, built in-container per architecture (D-03, D-06) |

## The isolation rules this experiment follows

The point of `experiments/` is that an experiment can be deleted in one
`rm -rf` and nothing else notices. Concretely:

1. **Nothing outside this directory imports anything inside it.** Enforced
   by `tests/test_isolation_boundary.py`, which scans `ros/`, `training/`
   and the repository's top-level shell entry points for a reference to
   this experiment's name, and is itself proven to catch a reference one
   directory level outside this experiment while allowing one written
   inside it.
2. **No shared build or test config knows about the experiment.**
   `ros/sim.sh`, `ros/dev.sh` and `ros/docker/compose.yaml` are untouched;
   this experiment's own `docker/compose.override.yaml` extends the running
   container instead, and never redeclares the base service's DDS/RMW
   settings. Enforced by `tests/test_compose_override.py`.
3. **The ROS packages this experiment needs live outside `ros/src/`**, so
   `ros/deploy.sh` -- which rsyncs `ros/src/` and builds
   `--packages-up-to wojtek_bringup` -- cannot ship them to the robot even
   by accident. `rai_interfaces` is imported into this experiment's own
   `ros_ws/`, never `ros/src/`. Enforced by `tests/test_repos_pin.py` and,
   for the general case, `tests/test_isolation_boundary.py`.
4. **Behaviour this experiment needs from RAI's own library is used
   through its public API, never copied or forked into this experiment's
   own code.** `wojtek_rai/topics.py` calls RAI's `ROS2Connector` directly
   rather than re-implementing ROS 2 topic discovery. No test enforces this
   one yet -- Phase 1 has too little of its own code for a fork to be worth
   guarding against; revisit once a later phase's tool registry gives this
   rule a real edge to test.
5. **One implementation of shared logic.** Phase 1 has a single Python
   package (`wojtek_rai/`) and no ROS nodes of its own yet, so there is
   nothing to hand-sync a copy of. A later phase that adds ROS nodes on top
   of this package must import from it rather than duplicating it; that is
   the point at which this rule needs its own test.

## What this experiment changes outside itself

| change | why it is not experiment plumbing |
|---|---|
| root `.env.example` (append placeholder vendor/tracing credential names) | This project keeps a single secrets file at the repository root rather than one per experiment (D-11); the placeholders are names only, never values |

## Secrets and credentials

Credentials come only from the gitignored root `.env` (`.env.example` is
its committed template); `run.sh` sources it and forwards only the
credential names a process already has set into the container, by name,
never by value. The committed `config.toml` carries model/vendor names
only -- its upstream schema has no credential field to fill in the first
place. `tests/test_no_secrets_in_config.py` scans every tracked file in
this experiment plus `.env.example` for secret-shaped values and for the
identity of private infrastructure, and is proven fail-first. Nothing in
this directory names a real host, address, login or alias; the second
development machine this project uses is referred to only as the remote
GPU dev box.
