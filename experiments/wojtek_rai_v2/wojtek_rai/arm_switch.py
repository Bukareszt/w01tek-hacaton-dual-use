"""The operator's arm/disarm switch for the panel.

Arming is a human action: the LLM has no arm tool and /wojtek/arm is on the
tools' forbidden list (limits.FORBIDDEN). The panel calls the service itself,
on the camera feed's node (spun on its own thread), and shows the robot's
answer. real_io refuses to arm unless the robot stands in the home pose, so
a refusal is normal and is reported verbatim, never raised.
"""

from __future__ import annotations

import time
from typing import Tuple

from std_srvs.srv import SetBool

ARM_SERVICE = "/wojtek/arm"          # real_io: targets reach the motors only while armed
POLICY_SERVICE = "/wojtek/enable"    # policy_node: the RL gait computes targets only while enabled
SERVICE_WAIT_S = 3.0
CALL_TIMEOUT_S = 5.0
POLL_S = 0.05


def call_set_bool(node, service: str, value: bool, timeout: float = CALL_TIMEOUT_S) -> Tuple[bool, str]:
    """Call a std_srvs/SetBool service; (success, message from the robot or the failure)."""
    client = node.create_client(SetBool, service)
    try:
        if not client.wait_for_service(timeout_sec=min(timeout, SERVICE_WAIT_S)):
            return False, f"{service} not available (is the robot stack up and the link alive?)"
        req = SetBool.Request()
        req.data = bool(value)
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                return False, f"{service} timed out after {timeout:.0f} s (link down?)"
            time.sleep(POLL_S)
        res = future.result()
        return bool(res.success), str(res.message)
    finally:
        # Never leave a client per click on the node.
        node.destroy_client(client)


def set_armed(node, armed: bool, timeout: float = CALL_TIMEOUT_S) -> Tuple[bool, str]:
    return call_set_bool(node, ARM_SERVICE, armed, timeout)


def set_policy_enabled(node, enabled: bool, timeout: float = CALL_TIMEOUT_S) -> Tuple[bool, str]:
    return call_set_bool(node, POLICY_SERVICE, enabled, timeout)
