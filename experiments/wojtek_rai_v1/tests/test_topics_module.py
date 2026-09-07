"""Model-free coverage of `wojtek_rai.topics`.

Covers the four behaviours this module promises:
  1. format_topics() renders a stable one-line-per-topic string with name + type
  2. format_topics([]) returns a non-empty "no topics" message, not ""
  3. importing wojtek_rai.topics never pulls rclpy into sys.modules
     (the rclpy/rai imports inside list_topics() are function-local)
  4. wojtek_rai.topics.main is callable and accepts an optional argv sequence

This suite needs no ROS 2, no LLM key, and no GPU (FOUND-06).
"""

import sys

import wojtek_rai.topics as topics


def test_format_topics_renders_name_and_type():
    rendered = topics.format_topics([("/cmd_vel", ["geometry_msgs/msg/Twist"])])
    assert "/cmd_vel" in rendered
    assert "geometry_msgs/msg/Twist" in rendered
    # one line per topic
    assert rendered.count("\n") == 0


def test_format_topics_renders_multiple_topics_one_line_each():
    rendered = topics.format_topics(
        [
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
            ("/camera/color/image_raw", ["sensor_msgs/msg/Image"]),
        ]
    )
    lines = rendered.splitlines()
    assert len(lines) == 2
    assert "/cmd_vel" in lines[0]
    assert "/camera/color/image_raw" in lines[1]


def test_format_topics_empty_input_is_a_non_empty_message():
    rendered = topics.format_topics([])
    assert rendered != ""
    assert "no topics" in rendered.lower()


def test_import_does_not_pull_in_rclpy():
    # If rai/rclpy were imported at module scope, they would already be in
    # sys.modules simply from `import wojtek_rai.topics` above -- this is
    # the load-bearing assertion for FOUND-06 (importable with no ROS 2
    # installed at all).
    assert "rclpy" not in sys.modules
    assert "rai" not in sys.modules


def test_main_is_callable_with_optional_argv():
    # Whether discovery actually succeeds depends on whether this process
    # has a live ROS 2 environment (it does when run.sh runs this suite
    # inside the container, even with no sim: ROS2Connector's own
    # TransformListener subscribes to /tf, so at minimum its own topics are
    # always visible) -- the contract this test enforces is only that
    # main() is callable, accepts an optional argv, and always returns an
    # int exit code rather than raising, per <behavior> in the plan.
    assert callable(topics.main)
    result = topics.main([])
    assert isinstance(result, int)

    result_no_args = topics.main()
    assert isinstance(result_no_args, int)
