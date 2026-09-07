"""Shared fixtures for this experiment's test suite.

This suite is model-free: no `rclpy`, no LLM key, no GPU (FOUND-06). Every
test here must be able to pass on a bare interpreter with none of those
present.
"""

from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    """Repository root, resolved the same way training/tests/unit does."""
    return Path(__file__).resolve().parents[3]


@pytest.fixture
def experiment_dir() -> Path:
    """This experiment's own root: experiments/wojtek_rai_v1/."""
    return Path(__file__).resolve().parents[1]
