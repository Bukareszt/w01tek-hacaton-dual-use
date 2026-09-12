# S3 — a panel E-STOP that works while the agent is mid-turn

Status: plan, not started. Stack item S3 on top of PR #11 (rai-nav-hardening);
one PR touching `experiments/wojtek_rai_v2/` only. Builds on
[rai-nav2-on-wojtek.md](rai-nav2-on-wojtek.md) (N4 = real robot) and the
README's Known gaps. `file:line` refer to branch `nav2-stack-plans`; paths
without a directory are `experiments/wojtek_rai_v2/wojtek_rai/`.

## 1. What happens today

- The turn runs on the Streamlit script thread: `app.py:193` calls
  `run_turn`, which loops over `graph.stream(...)` (`stream.py:82`). A click
  reruns the script, but a running script stops only at its next `st.*` call
  — `run_turn`'s callbacks (`stream.py:95-108`), never inside a tool.
- Inside a tool nothing yields: `walk` sleeps in its pulse loop for up to
  `MOVE_MAX_SECONDS` = 5 s (`tools.py:144-154`, `limits.py:96`); every Nav2
  tool (`navigate_to_pose`, `go_to_place`, `go_to_object` via
  `perception_tools.py:363`) blocks in `_wait(result_future,
  NAV_GOAL_TIMEOUT_S)` for up to 120 s (`nav_tools.py:174`, `limits.py:56`).
  Disarm/Stop clicks land after the tool returns (README.md:202-208).
- The operator buttons already bypass the agent: `_robot_button`
  (`app.py:126-134`) calls `arm_switch` on the camera feed's node, spun by its
  own daemon thread (`camera_feed.py:215-218`); `_call` polls (`arm_switch.py:26-42`).
- The LLM's `stop` has the right order: `cancel_active_nav_goal()` first,
  then `stop` on `/wojtek/nav_command` (`tools.py:252-263`,
  `nav_tools.py:62-79`) — Nav2 streams `/cmd_vel_nav` at 20 Hz and overwrites
  a text zero within 50 ms. A goal without a handle yet is marked
  `stop_requested` and cancelled on acceptance (`nav_tools.py:145-172`).
- `text_commander` re-arms its 2 s dead-man on every motion word and emits one
  zero Twist on `stop` (`ros/src/wojtek_teleop/wojtek_teleop/text_commander.py:54-76`);
  `policy_node` then holds that zero (`cmd_vel_timeout_s` default 0, PR #10).
- Disarm: `real_io` drops targets while not armed (`real_io_node.py:199-200`,
  `_srv_arm` `:337-340`); drives hold the last target. Policy off
  (`policy_node.py:219-226`) stops the tick (`:253-255`) and zeroes nothing.
- Cancellation in the installed stack (`wojtek_rai:jazzy`: rai-core 2.12.1,
  langgraph 1.2.11, streamlit 1.63.0): rai-core's only interrupt is
  `LangChainAgent._interrupt_event`, polled between stream chunks of its HRI
  run loop (`rai/agents/langchain/agent.py:205-206`), which the app never
  enters (direct mode, `agent.py:63-69`). LangGraph's sync executor cancels
  only tasks not yet started (`langgraph/pregel/_executor.py:59`). Tools run
  on the ToolNode's executor threads (`langgraph/prebuilt/tool_node.py:821-824`).
  Conclusion: no run cancellation to use; a process-wide flag the tools poll,
  plus a ROS-side stop that needs no tool at all.

## 2. Design

The LLM tool list does not change (`build_tools`, `tools.py:328-373`);
`/wojtek/arm` stays in `limits.FORBIDDEN` (`limits.py:76-84`).

1. `estop.py` — pure helper, no `st`, no rclpy at module scope.
   - `STOP = threading.Event()`, `request()/clear()/requested()`;
     `class EstopRequested(RuntimeError)` raised by tools. LangGraph's
     ToolNode turns it into an error ToolMessage; the ReAct loop ends on its
     own (embodiment rule "do not retry errors"; the recursion limit of 25
     bounds it, every further call is refused instantly).
   - `emergency_stop(node, publish, *, cancel_nav, disarm, disarm_after,
     burst_s=STOP_BURST_S, period_s=0.1, sleep=time.sleep,
     clock=time.monotonic) -> EstopReport`, in order:
     1. `STOP.set()`;
     2. `cancel_nav()` (= `nav_tools.cancel_active_nav_goal`; False is fine);
     3. `publish(limits.STOP_COMMAND)` every `period_s` for `burst_s`
        (`STOP_BURST_S = 2.5`, asserted `> limits.TEXT_COMMANDER_DEADMAN_S`):
        a motion pulse that left a tool thread just before the flag was seen
        re-arms the dead-man, so one zero is not enough;
     4. if `disarm_after`: `disarm()` (= `arm_switch.set_armed(node, False)`),
        answer recorded, never raised.
   - The nav-command publisher is created once at panel start on the feed
     node (a fresh DDS publisher drops its first messages, `tools.py:72-75`),
     cached with `st.cache_resource`. `publish` records
     `get_subscription_count() == 0` ("no text_commander listening") but
     still publishes; it never waits the way `_send` does.
2. Tools honour the flag (no tool added or removed):
   - `_NavCommandMixin._send` (`tools.py:89-110`): raise `EstopRequested`
     when `STOP.is_set()` and `command != limits.STOP_COMMAND`; the stop
     word passes, so the `finally` blocks (`:152-154`, `:218-220`) still work.
   - `_TimedNavMixin._navigate` (`nav_tools.py:127-143`): return "refused:
     emergency stop latched" before touching `_ACTIVE`; same check at
     `_send_and_wait` entry. `cancel_active_nav_goal` unchanged.
   - `_TriggerServiceTool._run` (`tools.py:269-284`): same refusal. `WaitForSecondsTool` is rai's; left alone.
3. The turn leaves the script thread: `turn_job.py`.
   - `TurnJob(graph, history)`: daemon thread running `run_turn` with
     `TurnEvents` that only `queue.put` tuples; the worker never touches
     `st.*` or `st.session_state` (ScriptRunContext is thread-local). Exposes
     `drain()`, `snapshot()` (finished steps, current text, partial calls),
     `done`, `result`, `error` (exception captured; `app.py:195-201` relies on it).
   - `app.py`: on a prompt, store the job in `st.session_state["turn"]` and
     return. The assistant area is a `@st.fragment(run_every=0.2)` while a
     job runs: drain, render `snapshot()` from scratch each tick (a fragment
     redraws itself; no incremental placeholders), on `done` move
     `result.messages` into `st.session_state.messages`, drop the job,
     `st.rerun(scope="app")`. `chat_input(..., disabled=<job running or
     STOP.is_set()>)`. `chat.py` keeps calling `run_turn` directly.
   - Sidebar (`app.py:163-173`) is main-script code, so a click is served
     within one rerun: E-STOP button (`type="primary"`, above the Robot
     panel), "disarm after stop" checkbox (default on), "clear e-stop"
     button, and the `EstopReport` lines (cancel yes/no, N stops over T s,
     listener yes/no, disarm answer). The sequence runs synchronously in the
     click handler (2.5 s), on the feed node like `arm_switch`.

Not doing: binding Streamlit to localhost (private network; `--server.address=127.0.0.1`
in `run.sh agent` is an option), twist_mux, a hardware stop.

## 3. Steps (about 15 h)

| # | step | h |
|---|---|---|
| 1 | `estop.py`: flag, exception, `emergency_stop`, `EstopReport`, burst assertion | 2 |
| 2 | Flag checks in `tools.py` (`_send`, `_TriggerServiceTool`) and `nav_tools.py` (`_navigate`, `_send_and_wait`) | 1.5 |
| 3 | `turn_job.py` (worker, queue, snapshot) | 2 |
| 4 | `app.py`: job in session state, polling fragment, disabled input, E-STOP / checkbox / clear, cached publisher | 3.5 |
| 5 | Tests (section 4) | 3 |
| 6 | README: replace the "not an e-stop" bullet (README.md:63-66) and the Known-gaps bullet (:202-208); state: software stop over WiFi through container and browser, the pad and the robot's disarm stay the real ones; N4 note: first robot session includes an E-STOP drill | 1 |
| 7 | Sim check (section 5), fix-ups, PR | 2 |

## 4. Tests (model-free, in the wojtek_rai image)

- `tests/test_estop.py` (fake node as `test_arm_switch.py:27-34`, fake
  clock/sleep, recording `publish`): order flag, cancel, stops, disarm; stops
  span `>= TEXT_COMMANDER_DEADMAN_S`; `disarm_after=False` calls no service;
  `cancel_nav` False or raising still reaches burst and disarm; a disarm
  refusal is reported, not raised; no listener is reported and the burst ran.
- `tests/test_tools.py`: `walk` with the flag set after the first pulse
  publishes no further motion word, one `stop`, raises `EstopRequested`;
  `StopTool._run` works with the flag set; `stand_up` refused; tool names
  from `build_tools` unchanged (`:136`), none named `estop`/`arm`.
- `tests/test_nav_timeout.py`: `_navigate` with the flag set creates no
  `ActionClient` (pattern `:187-193`); flag raised mid-`_send_and_wait`
  cancels on acceptance (reuse `_SlowNav2`, `:139-157`).
- `tests/test_turn_job.py`: fake `graph.stream` yielding recorded
  `("messages", ...)`/`("updates", ...)` tuples (shapes at `stream.py:82-108`):
  events in order, `done` with `result.messages`; a raising graph gives
  `error` and `done`; the caller is never blocked (join with timeout);
  `turn_job.py` imports no `streamlit`.
- Autouse fixture resets `estop.STOP` and calls `nav_tools._reset_active()`.

## 5. Verification

```bash
W=<worktree>
docker run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 -v "$W/experiments/wojtek_rai_v2:/exp" wojtek_rai:jazzy /entrypoint.sh python3 -m pytest -p pytest_timeout tests -q -p no:cacheprovider
git -C "$W" diff --check
# versions the plan relies on (must print 2.12.1 1.2.11 1.63.0)
docker run --rm wojtek_rai:jazzy /entrypoint.sh python3 -c "import importlib.metadata as m; print(m.version('rai-core'), m.version('langgraph'), m.version('streamlit'))"
# sim on the laptop: README "Run it" (ros/sim.sh, tunnel, agent), robot armed
./experiments/wojtek_rai_v2/run.sh agent
docker exec -i wojtek_rai /entrypoint.sh ros2 topic echo /cmd_vel --field linear.x   # second terminal
```
In the panel: (a) "walk forward for 5 seconds", E-STOP within 1 s: `/cmd_vel`
reads 0 within one text_commander tick of the first burst message, the walk's
tool result is the `EstopRequested` error, the turn ends, chat input stays
disabled until "clear e-stop"; (b) with `run.sh nav launch`: "go to the
hydrant", E-STOP: bt_navigator logs the cancel, `ros2 action list` shows no
goal, report says cancel yes; (c) checkbox off: `/wojtek/arm` not called; on:
"disarmed" in the report, ARMED banner gone; (d) the camera preview keeps
refreshing during a walk (the script no longer blocks).

## 6. Not verifiable without the robot

- Burst latency over the WiFi AP and whether the robot's `text_commander`
  sees it inside its dead-man; discovery delay of the once-created publisher
  after a link flap (`limits.py:101-104`).
- What the drives do on a mid-stride disarm (hold vs. sag) — why the disarm
  is a checkbox and the first robot session needs a drill with a hand on power
  (N4 sequence, `rai-nav2-on-wojtek.md:176-178`).
- Stop-burst-on-reconnect, pad interference (no twist_mux), Nav2 cancel timing
  on real odometry (`target:=real` has never run, README.md:194-201).

## 7. Stack notes

- Independent of S2, S4 and R2 in code: S3 edits `app.py`, `tools.py`,
  `nav_tools.py`, `test_tools.py`, `test_nav_timeout.py` and adds two
  modules; none of the other three touches those. Runs in parallel with S2.
- README is the only overlap: S3 replaces `:63-66` and `:202-208`, S4
  replaces the adjacent `:209-213`, S2/R2 rewrite `:194-201`; whichever
  lands second rebases the bullet list. PR #10 inserts 7 lines after `:86`,
  shifting every later README reference by +7.
- With S4 landed, `text_commander` writes `/cmd_vel_text` and its stop zero
  holds `/cmd_vel` only for the mux's 0.5 s text timeout; the design is
  unchanged because step 2 of `emergency_stop` (the Nav2 cancel) and the
  watchdog's zero are what stop Nav2, not the text zero. Section 5 (a)
  still reads `/cmd_vel`. `limits.NAV_COMMAND_TOPIC` is untouched by S4.
- `cmd_vel_timeout_s:=0.5` (last risk row) exists only once PR #10 merges.

## 8. Risks

| risk | handling |
|---|---|
| The flag stops tools, not the LLM: a few more tool calls before the turn ends | each refused in microseconds; recursion limit; chat input disabled until cleared |
| A tool thread pulses `forward` right after the first stop (race) | burst > dead-man; `_send` refuses motion words as soon as the flag is set |
| Fragment redraw cost while a turn runs (images in tool results) | render from snapshot at 0.2 s; tool text already cut at 4000 chars (`app.py:77`) |
| Worker touching `st.*` silently drops output | worker uses only the queue; test asserts no `streamlit` import in `turn_job.py` |
| Shared `st.cache_resource` graph: two tabs, two workers | same as today (each tab ran its own turn); one job per session via the disabled input |
| Disarm while a velocity is still latched in `policy_node` (timeout 0 by default) | burst zeroes `/cmd_vel` before the disarm; nav sessions launch with `cmd_vel_timeout_s:=0.5` (PR #10) |
| Operator reads "E-STOP" as a hardware stop | README wording (step 6); the pad and the robot's own disarm remain the real ones |
| No `text_commander` on the robot (subscriber count 0) | reported in the panel; cancel and disarm still run |
