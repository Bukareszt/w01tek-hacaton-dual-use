"""Guards over `ros/rai_interfaces.repos` and the `ros/src` overlay boundary.

`rai_interfaces` is a ROS 2 message package with no PyPI release, so its
reproducibility guarantee lives entirely in this `.repos` file (D-09). What
this suite checks is what this repository cares about: the pinned commit is
a concrete 40-character SHA (never a branch or a bare tag), the URL is the
real upstream repository, and nothing that came from this import -- or from
this experiment generally -- ever lands under `ros/src/`, which
`ros/deploy.sh` rsyncs to the physical robot.

This suite is model-free: no `rclpy`, no LLM key, no GPU, no network. It
asserts the shape of the committed pin, not that the commit still exists
upstream -- re-resolving the tag is a `run.sh build`-time / planning-time
concern, not something this test does on every run.
"""

import re
from pathlib import Path

import pytest
import yaml

# A concrete git commit SHA: exactly 40 lowercase hex characters. Never a
# branch name (`main`, `master`) and never a bare tag (`0.3.0`) -- both of
# those are moving or human-friendly references, not the commit itself.
COMMIT_SHA_RE = re.compile(r"^[0-9a-f]{40}$")

# The real upstream repository this project depends on -- guards against a
# typo'd fork or an unrelated repository being pinned instead.
EXPECTED_URL_RE = re.compile(r"^https://github\.com/RobotecAI/rai_interfaces(\.git)?$")


def repos_file(repo_root: Path) -> Path:
    return repo_root / "experiments" / "wojtek_rai_v1" / "ros" / "rai_interfaces.repos"


def test_repos_file_exists_and_parses(repo_root: Path):
    path = repos_file(repo_root)
    assert path.is_file(), f"missing {path}"
    data = yaml.safe_load(path.read_text())
    assert isinstance(data, dict), f"{path.name} did not parse into a mapping"
    assert "repositories" in data, f"{path.name} has no top-level 'repositories' key"


def _rai_interfaces_entry(repo_root: Path) -> dict:
    path = repos_file(repo_root)
    data = yaml.safe_load(path.read_text())
    repositories = data.get("repositories", {})
    assert "rai_interfaces" in repositories, (
        f"{path.name} has no 'rai_interfaces' entry in 'repositories': "
        f"{sorted(repositories)}"
    )
    return repositories["rai_interfaces"]


def test_version_is_a_commit_sha_not_a_branch_or_tag(repo_root: Path):
    entry = _rai_interfaces_entry(repo_root)
    version = entry.get("version", "")
    assert COMMIT_SHA_RE.match(version), (
        f"rai_interfaces 'version' must be a 40-character commit SHA, "
        f"got {version!r} in {repos_file(repo_root)}"
    )
    assert version not in {"main", "master"}, (
        f"rai_interfaces 'version' must never be a branch name, got {version!r}"
    )


def test_url_points_at_the_real_upstream_repository(repo_root: Path):
    entry = _rai_interfaces_entry(repo_root)
    url = entry.get("url", "")
    assert EXPECTED_URL_RE.match(url), (
        f"rai_interfaces 'url' must point at RobotecAI/rai_interfaces, "
        f"got {url!r} in {repos_file(repo_root)}"
    )


def test_no_rai_interfaces_or_experiment_source_under_ros_src(repo_root: Path):
    ros_src = repo_root / "ros" / "src"
    offenders = [
        p
        for p in ros_src.rglob("*")
        if "rai_interfaces" in p.name or "wojtek_rai_v1" in p.name
    ]
    assert not offenders, (
        f"found rai_interfaces/wojtek_rai_v1 paths under {ros_src}, which "
        f"ros/deploy.sh rsyncs to the robot: {offenders}"
    )
