"""Shared fixtures for this experiment's test suite.

This suite is model-free: no `rclpy`, no LLM key, no GPU (FOUND-06). Every
test here must be able to pass on a bare interpreter with none of those
present.
"""

from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    """Repository root.

    Ancestor-based resolution (`parents[3]`, the same way
    `training/tests/unit` does it) is correct on the host -- a possible
    future CI / `EXP_PY` path. Inside the `wojtek_robot` container (this
    suite's real, only execution path via `run.sh test`), only this
    experiment's own directory and `ros/src` are bind-mounted by default
    (D-02); `parents[3]` there resolves to `/ros2_ws`, which has neither
    `ros/` nor `training/` as a sibling -- a test built on that path (e.g.
    a check under `ros/src/`) would silently scan an empty, nonexistent
    directory and pass vacuously (T-01-17) rather than failing loudly.
    Plan 01-04 added a further mount at `/ros2_ws/repo_root` specifically
    so guard tests can see the real tree; prefer it when it is populated.
    """
    container_mount = Path("/ros2_ws/repo_root")
    if (container_mount / "ros").is_dir() and (container_mount / "training").is_dir():
        return container_mount
    return Path(__file__).resolve().parents[3]


@pytest.fixture
def experiment_dir() -> Path:
    """This experiment's own root: experiments/wojtek_rai_v1/."""
    return Path(__file__).resolve().parents[1]
