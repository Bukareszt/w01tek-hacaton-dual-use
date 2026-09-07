"""Secret-shape and private-infrastructure-identity guard over tracked config.

Pattern source: `training/tests/unit/test_job_scripts.py` (`FORBIDDEN_RE` /
`SITE_VALUE_RE` / `ABSOLUTE_PATH_RE`), read this session -- same shape
(module-level compiled regex constants, a `Path`-returning discovery
helper, one assert-with-diagnostic-message per check), retargeted here for
API-key shapes and for the identity of private infrastructure rather than
scheduler/host identity.

What this suite checks: every file `git ls-files` reports as tracked under
`experiments/wojtek_rai_v1/`, plus the repo-root `.env.example`, contains no
value shaped like a vendor/tracing credential (FOUND-04) and no value
identifying private infrastructure -- a dotted-quad IP address, a
`user@host` login form, or an `ssh`/`scp`/`rsync` invocation naming a
remote host (CLAUDE.md's public-repository rule).

What this suite deliberately does NOT check: whether a value is *actually*
a live credential or a real host -- only whether it is *shaped* like one.
This is model-free: no `rclpy`, no LLM key, no GPU, no network (FOUND-06).

Both scans are proven fail-first by the two `test_*_regex_detects_*` tests
below, which build a violating string from parts at runtime rather than
writing an example credential or host identity into this source file.

**Container execution note:** this suite normally runs inside the
`wojtek_robot` container via `run.sh test`, where only this experiment's
own directory (plus `ros/src`) is bind-mounted (D-02) -- there is no `.git`
anywhere inside that mount, so `git ls-files` cannot run there.
`tracked_config_files()` detects this (no `.git` reachable by walking
upward from this file) and falls back to a plain directory walk that
excludes exactly what this experiment's own `.gitignore` excludes, which
can only over-scan relative to `git ls-files`, never under-scan. The
root `.env.example` is similarly unreachable inside that mount by default;
`docker/compose.override.yaml` adds one additional read-only bind mount of
just that single file (already public, secret-free by design -- it is a
template of placeholder names) at the container path this fallback expects,
so the scan can still cover it.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
THIS_FILE = Path(__file__).resolve()

# Paths this experiment's own .gitignore excludes (generated/arch-specific,
# rebuilt independently on each machine) -- mirrored here, not parsed from
# the file, since the fallback below only runs where git itself is
# unreachable. Directory names ignored everywhere regardless of position.
_GITIGNORED_PREFIXES = (
    ".venv/",
    ".tools/",
    ".uv-cache/",
    "ros_ws/build/",
    "ros_ws/install/",
    "ros_ws/log/",
    "ros_ws/src/",
)
_IGNORED_DIR_NAMES = {"__pycache__", ".pytest_cache", ".git"}

# Common cloud API key / token shapes. Extend as new vendors are added
# (HRI-03).
SECRET_SHAPE_RE = re.compile(
    r"sk-[A-Za-z0-9]{16,}"  # OpenAI-style
    r"|AKIA[0-9A-Z]{16}"  # AWS access key id
    r"|AIza[0-9A-Za-z_-]{35}"  # Google API key
    r"|ey[A-Za-z0-9_-]{10,}\."  # JWT-shaped
    r"|(?:pk|sk)_(?:live|test)_[A-Za-z0-9]{16,}"  # Stripe-style live/test key
)

# A hard-coded credential/secret field with a non-empty value -- belt and
# suspenders alongside SECRET_SHAPE_RE for a key whose value doesn't happen
# to match one of the shapes above.
POPULATED_KEY_FIELD_RE = re.compile(
    r"(?i)\b(api_key|api_token|secret)\s*[=:]\s*[\"'][^\"']+[\"']"
)

# The identity of private infrastructure: a dotted-quad IP address, a
# user@host login form (also catches a personal email address), or an
# ssh/scp/rsync invocation naming a remote host. Mirrors the structure of
# training/tests/unit/test_job_scripts.py's SITE_VALUE_RE, retargeted from
# scheduler/host identity to this repository's public-repo rule
# (CLAUDE.md §"Repository map and boundaries": no hostnames, IPs, logins,
# SSH aliases, or personal emails in a tracked file).
PRIVATE_IDENTITY_RE = re.compile(
    r"\b(?:\d{1,3}\.){3}\d{1,3}\b"  # dotted-quad IP address
    r"|[A-Za-z0-9_.+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}"  # user@host / email
    r"|\b(?:ssh|scp|rsync)\b[^\n]*@\S+"  # remote-host invocation
)


def _find_git_root(start: Path) -> Path | None:
    """Walk upward from `start` looking for a `.git` entry.

    Returns `None` inside the `wojtek_robot` container, where only this
    experiment's directory is bind-mounted and no ancestor has `.git`.
    """
    for candidate in (start, *start.parents):
        if (candidate / ".git").exists():
            return candidate
    return None


def _walk_experiment_dir_excluding_generated(experiment_dir: Path) -> list[Path]:
    """Fallback discovery when no `.git` is reachable (see module docstring).

    Filters out exactly the paths this experiment's own `.gitignore`
    excludes, plus universal noise directories. The result can only
    over-approximate `git ls-files`'s view (a few more untracked-but-not-
    ignored files), never under-approximate it -- the safe direction for a
    security guard.
    """
    results = []
    for path in experiment_dir.rglob("*"):
        if not path.is_file():
            continue
        rel_parts = path.relative_to(experiment_dir).parts
        if any(part in _IGNORED_DIR_NAMES for part in rel_parts[:-1]):
            continue
        rel = path.relative_to(experiment_dir).as_posix()
        if rel.startswith(_GITIGNORED_PREFIXES):
            continue
        results.append(path)
    return results


def tracked_config_files() -> list[Path]:
    """Every git-tracked file under this experiment, plus root .env.example.

    Discovered with `git ls-files` when `.git` is reachable, so untracked
    local scratch files are correctly out of scope -- a developer's own
    uncommitted experiments must never fail this suite. Falls back to
    `_walk_experiment_dir_excluding_generated()` when it is not (see module
    docstring). This guard test's own source is excluded either way: it
    necessarily contains the detection patterns themselves as code, not a
    real leak.
    """
    git_root = _find_git_root(EXPERIMENT_DIR)
    if git_root is not None:
        result = subprocess.run(
            ["git", "ls-files", str(EXPERIMENT_DIR)],
            cwd=git_root,
            capture_output=True,
            text=True,
            check=True,
        )
        tracked = [
            git_root / line for line in result.stdout.splitlines() if line.strip()
        ]
        env_example = git_root / ".env.example"
    else:
        tracked = _walk_experiment_dir_excluding_generated(EXPERIMENT_DIR)
        env_example = REPO_ROOT / ".env.example"

    assert tracked, (
        f"no tracked files discovered under {EXPERIMENT_DIR} -- "
        "this suite must never pass by scanning an empty set"
    )
    assert env_example.is_file(), (
        f"{env_example} not found -- cannot verify FOUND-04 without it"
    )
    tracked = [p for p in tracked if p.resolve() != THIS_FILE]
    tracked.append(env_example)
    return tracked


def test_no_secret_shaped_values_in_tracked_files():
    for path in tracked_config_files():
        text = path.read_text()
        hits = SECRET_SHAPE_RE.findall(text)
        assert not hits, f"{path}: contains a secret-shaped value: {hits}"
        field_hits = POPULATED_KEY_FIELD_RE.findall(text)
        assert not field_hits, (
            f"{path}: has a populated credential field: {field_hits}"
        )


def test_no_private_infrastructure_identity_in_tracked_files():
    for path in tracked_config_files():
        if path.name == "uv.lock":
            # Generated lockfile: dotted package-version strings (e.g.
            # opencv-python-headless's "4.11.0.86") are indistinguishable
            # from a dotted-quad IP by shape alone, and nobody hand-writes
            # a host identity into a machine-generated lockfile. Still
            # covered by the secret-shape scan above.
            continue
        text = path.read_text()
        hits = PRIVATE_IDENTITY_RE.findall(text)
        assert not hits, (
            f"{path}: names private infrastructure (IP/login/SSH form): {hits}"
        )


def test_secret_shape_regex_detects_a_synthetic_openai_style_key():
    # Built from parts at runtime -- never a literal credential-shaped
    # string in this source file.
    synthetic = "sk-" + "a" * 20
    assert SECRET_SHAPE_RE.search(synthetic), (
        "SECRET_SHAPE_RE failed to detect a synthetic OpenAI-style key -- "
        "the guard would not catch a real one either"
    )


def test_secret_shape_regex_detects_a_synthetic_aws_access_key_id():
    synthetic = "AKIA" + "B" * 16
    assert SECRET_SHAPE_RE.search(synthetic), (
        "SECRET_SHAPE_RE failed to detect a synthetic AWS access key id"
    )


def test_private_identity_regex_detects_a_synthetic_dotted_quad_ip():
    synthetic = ".".join(["10", "42", "0", "2"])
    assert PRIVATE_IDENTITY_RE.search(synthetic), (
        "PRIVATE_IDENTITY_RE failed to detect a synthetic dotted-quad IP"
    )


def test_private_identity_regex_detects_a_synthetic_login_form():
    synthetic = "user" + "@" + "example.com"
    assert PRIVATE_IDENTITY_RE.search(synthetic), (
        "PRIVATE_IDENTITY_RE failed to detect a synthetic user@host form"
    )


def test_private_identity_regex_detects_a_synthetic_remote_shell_invocation():
    synthetic = "rsync -av ./ " + "user" + "@" + "host.example.com" + ":/dest"
    assert PRIVATE_IDENTITY_RE.search(synthetic), (
        "PRIVATE_IDENTITY_RE failed to detect a synthetic rsync invocation"
    )
