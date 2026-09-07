"""Isolation-boundary guard: nothing outside this directory imports it, and
`ros/deploy.sh` cannot ship it to the robot.

What this repository cares about, per CLAUDE.md's `experiments/` rule and
the sibling experiment's isolation rules (FOUND-01): (1) no file under
`ros/`, `training/` or a repository-root shell entry point references this
experiment by name; (2) no path under `ros/src/` -- the tree
`ros/deploy.sh` rsyncs to the robot -- matches this experiment's name or
`rai`; (3) every local source `ros/deploy.sh`'s rsync invocations read from
resolves inside `ros/` itself, never outside it. What this test
deliberately does NOT check: whether a name match is a real Python import
vs. a comment or a string -- a name match anywhere outside this directory
is already the violation FOUND-01 forbids, regardless of syntax.

`.planning/` is deliberately excluded from every scan in this file:
planning documents (PLAN.md/SUMMARY.md/RESEARCH.md, ...) legitimately name
this experiment throughout its own phase directory, and scanning them would
make this guard fail on the very commits that describe the experiment.
None of this file's scans include `.planning/` in their target list.

This suite is model-free: no `rclpy`, no LLM key, no GPU, no network
(FOUND-06) -- `grep` and the standard library only.
"""

from __future__ import annotations

import os
import re
import shlex
import subprocess
from pathlib import Path

import pytest

def _resolve_repo_root() -> Path:
    """The real repository root.

    Ancestor-based resolution (`parents[3]`) is correct when this file runs
    from the host's own checkout -- a possible future CI / `EXP_PY` path.
    Inside the `wojtek_robot` container (this suite's real, only execution
    path via `run.sh test`), only this experiment's own directory and
    `ros/src` are bind-mounted by default (D-02); `parents[3]` there
    resolves to `/ros2_ws`, which has neither `ros/` nor `training/` as a
    sibling. `docker/compose.override.yaml` adds one further mount, at
    `/ros2_ws/repo_root`, specifically so this guard can see the trees
    FOUND-01 requires it to scan (and write the fail-first boundary probes
    below) -- prefer it when it is actually populated.
    """
    container_mount = Path("/ros2_ws/repo_root")
    if (container_mount / "ros").is_dir() and (container_mount / "training").is_dir():
        return container_mount
    return Path(__file__).resolve().parents[3]


REPO_ROOT = _resolve_repo_root()
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
EXPERIMENT_NAME = EXPERIMENT_DIR.name  # "wojtek_rai_v1"

# A whole-word match on the experiment's own directory name. Word-bounded so
# a longer, unrelated identifier that merely contains this string as a
# substring is not a false positive.
NAME_RE = rf"\b{re.escape(EXPERIMENT_NAME)}\b"

# The two hard production trees FOUND-01 protects: a reference from either
# would make this experiment a dependency of production code.
PRODUCTION_ROOTS = ("ros", "training")

# `--exclude` is the one rsync flag in ros/deploy.sh that takes a value
# (the exclude pattern) on the next token -- that value is not a source
# path and must not be mistaken for one.
_RSYNC_FLAGS_WITH_VALUE = {"--exclude"}


def _grep_rl(pattern: str, targets: list[str], exclude_dir: str | None = None) -> list[str]:
    """Run `grep -rl -E pattern` over repo-root-relative `targets`.

    Returns the matching file paths (relative to REPO_ROOT). Asserts at
    least one target actually exists on disk before running grep: grep's
    own "no matches" (exit 1) and "nothing to search" cases are otherwise
    indistinguishable, and a guard that silently scans an empty set is
    worse than no guard at all.
    """
    existing = [t for t in targets if (REPO_ROOT / t).exists()]
    assert existing, (
        f"no scan targets exist among {targets} under {REPO_ROOT} -- "
        "this guard must never report a clean scan of an empty set"
    )
    cmd = ["grep", "-rl", "-E", pattern]
    if exclude_dir:
        cmd.append(f"--exclude-dir={exclude_dir}")
    cmd += existing
    result = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    # grep: 0 = matches found, 1 = no matches (clean), 2 = a real error
    # (bad pattern, unreadable target, ...) -- exit 2 must not read as clean.
    assert result.returncode in (0, 1), (
        f"grep failed (exit {result.returncode}) scanning {targets}: {result.stderr}"
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def _rsync_source_resolves_inside_ros(source: str) -> bool:
    """Substitute `${HERE}` (deploy.sh's own directory, i.e. `ros/`) and
    check the *normalized* result stays inside `ros/`.

    A plain `source.startswith("${HERE}")` string check is not enough: a
    source written as `${HERE}/../training/` also starts with `${HERE}` but
    normalizes to `training/`, escaping `ros/` entirely -- exactly the kind
    of widening this test exists to catch. Any source that does not even
    start with `${HERE}` (a literal absolute path, or a bare relative path
    resolved against deploy.sh's own cwd rather than its directory) is
    rejected outright.
    """
    if not source.startswith("${HERE}"):
        return False
    substituted = "ros" + source[len("${HERE}") :]
    normalized = os.path.normpath(substituted)
    return normalized == "ros" or normalized.startswith("ros" + os.sep)


def _rsync_source_tokens(tokens: list[str]) -> list[str]:
    """Extract rsync's non-flag arguments from an already-tokenized command."""
    sources = []
    skip_next = False
    for token in tokens[1:]:  # tokens[0] is "rsync" itself
        if skip_next:
            skip_next = False
            continue
        if token in _RSYNC_FLAGS_WITH_VALUE:
            skip_next = True
            continue
        if token.startswith("-"):
            continue
        sources.append(token)
    return sources


def test_empty_scan_set_is_rejected():
    """A scan whose targets don't exist must fail loudly, not report clean."""
    with pytest.raises(AssertionError, match="no scan targets exist"):
        _grep_rl(NAME_RE, ["definitely-not-a-real-directory-xyz"])


def test_production_roots_and_top_level_scripts_exist_and_are_non_empty():
    """The trees this guard depends on must themselves be real and populated.

    A misconfigured REPO_ROOT (e.g. resolved one level off) would otherwise
    make every other test in this file pass vacuously.
    """
    for root in PRODUCTION_ROOTS:
        path = REPO_ROOT / root
        assert path.is_dir(), f"expected production tree {path} to exist"
        assert any(path.rglob("*")), f"{path} exists but contains no files"


def test_no_reference_to_experiment_name_in_production_trees():
    """Nothing under ros/ or training/ -- or a repo-root shell script --
    references this experiment by name (FOUND-01's import-boundary half).
    """
    targets = list(PRODUCTION_ROOTS) + [p.name for p in REPO_ROOT.glob("*.sh")]
    hits = _grep_rl(NAME_RE, targets)
    assert not hits, (
        f"production tree references {EXPERIMENT_NAME!r}, making it a "
        f"potential dependency of production code: {hits}"
    )


def test_no_path_under_ros_src_matches_experiment_name_or_rai():
    """No path under ros/src/ -- the tree ros/deploy.sh rsyncs to the robot
    and builds via `--packages-up-to wojtek_bringup` -- names this
    experiment or RAI, even by accident.
    """
    ros_src = REPO_ROOT / "ros" / "src"
    rai_word_re = re.compile(r"(?i)\brai\b")
    offenders = [
        p
        for p in ros_src.rglob("*")
        if EXPERIMENT_NAME in p.name or rai_word_re.search(p.name)
    ]
    assert not offenders, (
        f"found experiment/RAI-named paths under {ros_src}, which "
        f"ros/deploy.sh rsyncs to the robot: {offenders}"
    )


def test_reference_inside_experiment_dir_is_allowed():
    """The boundary contract, positive control: a reference to this
    experiment's own name, written inside its own directory, must not trip
    this guard -- the directory is the boundary, not the mere string.
    """
    probe = EXPERIMENT_DIR / "_tmp_isolation_boundary_probe.txt"
    probe.write_text(f"see {EXPERIMENT_NAME} for details\n")
    try:
        hits = _grep_rl(NAME_RE, ["experiments"], exclude_dir=EXPERIMENT_NAME)
        assert not hits, (
            f"a reference inside {EXPERIMENT_DIR} must not be flagged "
            f"when the scan excludes the experiment's own directory: {hits}"
        )
    finally:
        probe.unlink()


def test_reference_one_level_outside_experiment_dir_is_rejected():
    """The boundary contract, negative control: the same reference, written
    one directory level outside (a sibling under experiments/, not inside
    wojtek_rai_v1/ itself), MUST trip the guard.
    """
    probe = REPO_ROOT / "experiments" / "_tmp_isolation_boundary_probe.txt"
    probe.write_text(f"see {EXPERIMENT_NAME} for details\n")
    try:
        hits = _grep_rl(NAME_RE, ["experiments"], exclude_dir=EXPERIMENT_NAME)
        assert hits, (
            "a reference one directory level outside the experiment "
            "(a sibling under experiments/) must be caught, not silently "
            "excluded along with the experiment's own directory"
        )
    finally:
        probe.unlink()


def test_deploy_sh_rsync_sources_all_resolve_inside_ros():
    """Every local source ros/deploy.sh's rsync invocations read from
    resolves inside ros/, so ros/deploy.sh can never ship this experiment
    to the robot even via a future change that widens its reach.

    ros/deploy.sh's own `HERE="$(cd "$(dirname "$0")" && pwd)"` is ros/
    itself, since the script lives at ros/deploy.sh. A source argument
    written as `"${HERE}/..."` therefore always resolves inside ros/ by
    construction; a literal absolute path or a `../` escape would not, and
    is exactly the kind of change this test exists to catch.
    """
    deploy_sh = REPO_ROOT / "ros" / "deploy.sh"
    assert deploy_sh.is_file(), f"missing {deploy_sh}"
    # Join backslash-continued lines into single logical lines before
    # scanning for rsync invocations -- ros/deploy.sh wraps its longer rsync
    # calls across several lines.
    logical = re.sub(r"\\\n\s*", " ", deploy_sh.read_text())
    rsync_lines = [
        line
        for line in logical.splitlines()
        if re.search(r"\brsync\b", line) and not line.strip().startswith("#")
    ]
    assert rsync_lines, f"no rsync invocation found in {deploy_sh} -- cannot verify its reach"

    checked_any_source = False
    for line in rsync_lines:
        tokens = shlex.split(line, posix=True)
        if not tokens or tokens[0] != "rsync":
            continue  # e.g. an `echo "... rsync ..."` line mentioning the word
        sources = [t for t in _rsync_source_tokens(tokens) if "${RPI_HOST}" not in t]
        for source in sources:
            checked_any_source = True
            assert _rsync_source_resolves_inside_ros(source), (
                f"{deploy_sh.name}: rsync source {source!r} does not "
                "resolve inside ros/ once ${HERE} (deploy.sh's own "
                "directory) is substituted and the path is normalized"
            )
    assert checked_any_source, f"found rsync line(s) in {deploy_sh} but no source argument to check"
