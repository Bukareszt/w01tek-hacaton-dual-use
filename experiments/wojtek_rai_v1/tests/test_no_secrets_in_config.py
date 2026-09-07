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
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]

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


def tracked_config_files() -> list[Path]:
    """Every git-tracked file under this experiment, plus root .env.example.

    Discovered with `git ls-files` so untracked local scratch files are
    correctly out of scope -- a developer's own uncommitted experiments
    must never fail this suite.
    """
    result = subprocess.run(
        ["git", "ls-files", str(EXPERIMENT_DIR)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    tracked = [
        REPO_ROOT / line for line in result.stdout.splitlines() if line.strip()
    ]
    assert tracked, (
        f"git ls-files returned nothing under {EXPERIMENT_DIR} -- "
        "this suite must never pass by scanning an empty set"
    )
    tracked.append(REPO_ROOT / ".env.example")
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
