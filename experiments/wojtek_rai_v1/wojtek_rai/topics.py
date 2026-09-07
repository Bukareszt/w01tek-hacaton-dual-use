"""ROS 2 topic discovery through RAI's own connector.

This is the thinnest possible exercise of the exact import chain every later
phase's ROS 2 tools sit on: `rai.communication.ros2.ROS2Connector`, not a
hand-rolled `rclpy` wrapper and not a shell-out to `ros2 topic list`.

The whole module is read-only against the ROS graph: no publisher, no
service client, no action client. `rclpy` and `rai` are imported inside
`list_topics()`, not at module scope, so `import wojtek_rai.topics` succeeds
on an interpreter with no ROS 2 on `PYTHONPATH` (FOUND-06) -- this is what
lets the module be imported and unit-tested on a machine with no ROS 2
installed at all.
"""

from __future__ import annotations

import sys
from collections.abc import Iterable, Sequence

# One row per topic: (topic_name, [message_type, ...]).
TopicRow = tuple[str, list[str]]


def list_topics() -> list[TopicRow]:
    """Return the live ROS 2 graph's topics as (name, types) rows.

    Opens a `rai.communication.ros2.ROS2Connector` against the process's
    current ROS 2 environment (domain id, RMW implementation, DDS config --
    all inherited from whatever environment this process was started in;
    see `run.sh`'s `container`/`agent-topics` targets, which run this inside
    the `wojtek_robot` container via `docker exec` so those variables are
    actually set), queries `get_topics_names_and_types()`, and shuts the
    connector down again.

    The `rai`/`rclpy` imports are deliberately function-local -- see the
    module docstring.
    """
    from rai.communication.ros2 import ROS2Connector

    connector = ROS2Connector()
    try:
        return list(connector.get_topics_names_and_types())
    finally:
        connector.shutdown()


def format_topics(rows: Iterable[TopicRow]) -> str:
    """Format topic rows as one line per topic: `<name>  <types>`.

    Pure function, no I/O. Returns a distinct, non-empty message when `rows`
    is empty rather than an empty string, so a caller printing the result
    never mistakes "no topics" for "nothing ran".
    """
    rows = list(rows)
    if not rows:
        return "No topics found."
    return "\n".join(f"{name}  {list(types)}" for name, types in rows)


def main(argv: Sequence[str] | None = None) -> int:
    """Print the live simulation's topic list. Returns a process exit code.

    Accepts an optional argv sequence (unused today, kept so this matches
    the standard `main(argv=None)` entry-point shape other Wojtek CLI
    modules use). Discovery failures are reported with a readable message
    and a non-zero exit code rather than a raw traceback.
    """
    del argv
    try:
        rows = list_topics()
    except Exception as exc:  # noqa: BLE001 -- report to the caller, don't crash silently
        print(f"topic discovery failed: {exc}", file=sys.stderr)
        return 1
    print(format_topics(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
