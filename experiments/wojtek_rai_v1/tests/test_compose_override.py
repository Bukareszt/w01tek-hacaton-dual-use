"""Compose-override shape guard (FOUND-03's static half).

What this repository cares about, per D-02/D-04: the override extends the
same service the base `ros/docker/compose.yaml` declares (read from that
file, not hardcoded, so a rename there surfaces here); it declares no
service-level `environment:` mapping, because the DDS/RMW settings
(`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`) must be inherited
unmodified from the base service, never redeclared and silently diverged;
and it still declares a volume mounting this experiment's own directory at
its container path, with a source that resolves -- against the directory
Compose actually resolves relative paths from, the base compose file's
own directory, not this override file's own directory -- to the experiment
directory. What this test deliberately does NOT check: how many *other*
volumes the override declares. Two later, independently-justified
deviations (`.env.example` in plan 01-03, the read-write repo-root mount in
plan 01-04, both documented inline in the override) add auxiliary mounts
this file does not need to know about by name; only the experiment's own
mount and the absence of an `environment:` block are load-bearing here.

This suite is model-free: a static `yaml.safe_load` of two already-
committed files, no network, no `rclpy`, no LLM key, no GPU (FOUND-06).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml


def _resolve_repo_root() -> Path:
    """The real repository root.

    Ancestor-based resolution (`parents[3]`) is correct on the host -- a
    possible future CI / `EXP_PY` path. Inside the `wojtek_robot` container
    (this suite's real, only execution path via `run.sh test`), only this
    experiment's own directory and `ros/src` are bind-mounted by default
    (D-02); `parents[3]` there resolves to `/ros2_ws`, which has no `ros/`
    as a sibling, so `ros/docker/compose.yaml` -- the base file this test
    reads the service name from -- would not be reachable at all. Plan
    01-04 added a further mount at `/ros2_ws/repo_root` specifically so
    guard tests can see the real tree; prefer it when it is populated.
    Mirrors `conftest.py`'s `repo_root` fixture and
    `test_isolation_boundary.py`'s identical helper.
    """
    container_mount = Path("/ros2_ws/repo_root")
    if (container_mount / "ros").is_dir() and (container_mount / "training").is_dir():
        return container_mount
    return Path(__file__).resolve().parents[3]


REPO_ROOT = _resolve_repo_root()
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
OVERRIDE_PATH = EXPERIMENT_DIR / "docker" / "compose.override.yaml"
BASE_COMPOSE_PATH = REPO_ROOT / "ros" / "docker" / "compose.yaml"

# The container path this experiment's own bind mount must target -- see
# run.sh's CONTAINER_EXP_DIR constant, which every container-side Python
# invocation in this experiment assumes.
EXPERIMENT_CONTAINER_PATH = "/ros2_ws/experiments/wojtek_rai_v1"


def _load_yaml(path: Path) -> dict:
    assert path.is_file(), f"missing {path}"
    text = path.read_text()
    assert text.strip(), f"{path} is empty"
    data = yaml.safe_load(text)
    assert isinstance(data, dict), f"{path.name} did not parse into a mapping"
    return data


def _base_service_name() -> str:
    """The container service name, read from the base compose file -- never
    hardcoded, so a rename there surfaces here rather than silently going
    stale.
    """
    base = _load_yaml(BASE_COMPOSE_PATH)
    services = base.get("services", {})
    assert services, f"{BASE_COMPOSE_PATH.name} declares no services"
    assert len(services) == 1, (
        f"{BASE_COMPOSE_PATH.name} declares more than one service "
        f"({sorted(services)}) -- this test assumes exactly one"
    )
    return next(iter(services))


def _override_service(override_data: dict, service_name: str) -> dict:
    services = override_data.get("services", {})
    assert service_name in services, (
        f"{OVERRIDE_PATH.name} does not extend the base service "
        f"{service_name!r} -- found {sorted(services)}"
    )
    return services[service_name]


def test_override_extends_the_same_service_the_base_compose_file_declares():
    service_name = _base_service_name()
    override_data = _load_yaml(OVERRIDE_PATH)
    services = override_data.get("services", {})
    assert list(services) == [service_name], (
        f"{OVERRIDE_PATH.name} must declare exactly the base service "
        f"{service_name!r}, found {sorted(services)}"
    )


def test_override_declares_no_environment_mapping():
    """D-04: ROS_DOMAIN_ID, RMW_IMPLEMENTATION and CYCLONEDDS_URI must be
    inherited unmodified from the base service. Redeclaring any of them
    here -- even to the same value -- risks a future edit silently
    diverging this experiment's DDS domain from the simulation it needs to
    discover topics on.
    """
    service_name = _base_service_name()
    override_data = _load_yaml(OVERRIDE_PATH)
    service = _override_service(override_data, service_name)
    assert "environment" not in service, (
        f"{OVERRIDE_PATH.name}'s {service_name!r} service redeclares "
        "'environment' -- DDS/RMW settings must be inherited from the base "
        "compose file, never restated here (D-04)"
    )


def test_override_declares_a_volumes_list():
    service_name = _base_service_name()
    override_data = _load_yaml(OVERRIDE_PATH)
    service = _override_service(override_data, service_name)
    volumes = service.get("volumes")
    assert isinstance(volumes, list) and volumes, (
        f"{OVERRIDE_PATH.name}'s {service_name!r} service declares no "
        "'volumes' list"
    )


def _experiment_volume_entries(volumes: list[str]) -> list[tuple[str, str]]:
    """Every volume entry whose target is this experiment's container path.

    Returns (source, target) pairs. A "source:target" or
    "source:target:ro"-shaped short syntax entry, per Compose's volume
    string format.
    """
    entries = []
    for entry in volumes:
        parts = entry.split(":")
        if len(parts) < 2:
            continue
        source, target = parts[0], parts[1]
        if target == EXPERIMENT_CONTAINER_PATH:
            entries.append((source, target))
    return entries


def test_experiment_bind_mount_target_and_source_are_correct():
    """The one load-bearing volume entry: this experiment's own directory,
    mounted at its container path, with a source that resolves -- against
    the base compose file's own directory, which is where Compose actually
    resolves a multi-file stack's relative paths, NOT this override file's
    own directory -- to the experiment directory on disk.
    """
    service_name = _base_service_name()
    override_data = _load_yaml(OVERRIDE_PATH)
    service = _override_service(override_data, service_name)
    volumes = service.get("volumes", [])
    matches = _experiment_volume_entries(volumes)
    assert matches, (
        f"{OVERRIDE_PATH.name} declares no volume whose target is "
        f"{EXPERIMENT_CONTAINER_PATH!r} -- the experiment bind mount is "
        "missing"
    )
    assert len(matches) == 1, (
        f"{OVERRIDE_PATH.name} declares more than one volume targeting "
        f"{EXPERIMENT_CONTAINER_PATH!r}: {matches}"
    )
    source, _ = matches[0]
    base_compose_dir = BASE_COMPOSE_PATH.resolve().parent
    resolved_source = (base_compose_dir / source).resolve()
    # `os.path.samefile`, not `==` on the resolved `Path`s: inside the
    # wojtek_robot container, this experiment's own directory is visible at
    # two different absolute paths through two different bind mounts of
    # the same host directory (D-02's own mount, and plan 01-04's
    # repo_root mount that REPO_ROOT prefers) -- lexical `Path` equality
    # would see two different strings and wrongly fail even when the
    # source is correct. `samefile` compares the underlying (dev, inode),
    # which is genuinely identical for two bind-mounted views of the same
    # host directory (verified against this exact container).
    assert resolved_source.exists() and EXPERIMENT_DIR.exists(), (
        f"cannot compare {resolved_source} and {EXPERIMENT_DIR} -- one of "
        "them does not exist"
    )
    assert os.path.samefile(resolved_source, EXPERIMENT_DIR), (
        f"{OVERRIDE_PATH.name}: volume source {source!r}, resolved against "
        f"{base_compose_dir} (the base compose file's own directory, the "
        "way Compose actually resolves a multi-file stack's relative "
        f"paths), is {resolved_source}, not the experiment directory "
        f"{EXPERIMENT_DIR}"
    )


def test_volume_source_resolved_against_overrides_own_directory_would_be_wrong():
    """Fail-first control for the resolution-base mistake this test exists
    to catch: resolving the SAME source string against the override file's
    OWN directory (docker/) -- the easy, wrong way to read a relative path
    -- must NOT land on the experiment directory. If it did, the real bug
    (a source written relative to the wrong base) would be undetectable by
    string comparison alone.
    """
    service_name = _base_service_name()
    override_data = _load_yaml(OVERRIDE_PATH)
    service = _override_service(override_data, service_name)
    volumes = service.get("volumes", [])
    matches = _experiment_volume_entries(volumes)
    assert matches, "no experiment volume entry to check"
    source, _ = matches[0]
    wrong_base = OVERRIDE_PATH.resolve().parent  # docker/, not ros/docker/
    wrongly_resolved = (wrong_base / source).resolve()
    same = wrongly_resolved.exists() and EXPERIMENT_DIR.exists() and os.path.samefile(
        wrongly_resolved, EXPERIMENT_DIR
    )
    assert not same, (
        "resolving the volume source against the override file's own "
        "directory landed on the experiment directory by coincidence -- "
        "this test's resolution-base distinction is not actually being "
        "exercised; rewrite the source path so the two bases diverge"
    )


def test_missing_experiment_volume_is_rejected():
    """Fail-first control: an override with no matching volume entry at all
    must be caught, not silently treated as 'nothing to check'.
    """
    empty_service = {"volumes": ["../../.env.example:/ros2_ws/.env.example:ro"]}
    assert not _experiment_volume_entries(empty_service["volumes"])
