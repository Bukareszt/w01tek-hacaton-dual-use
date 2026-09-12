"""The follow arithmetic, with no ROS in it.

Nothing in this package imports rclpy or any message type.  Every module here
takes numbers and dicts and gives numbers and dicts back, which is what lets
the whole of it be tested on a laptop with numpy and nothing else.  The node in
`follow_node.py` is the only file that knows ROS exists.
"""
