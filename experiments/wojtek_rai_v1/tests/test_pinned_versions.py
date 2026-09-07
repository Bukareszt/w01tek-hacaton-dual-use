"""Version-pinning guard over `pyproject.toml` and `uv.lock` (FOUND-02).

What this repository cares about: `rai-core` and `rai-whoami` are pinned to
an exact version in `pyproject.toml`, never a range; their transitives --
LangChain, LangGraph and every distribution under those two ecosystems --
carry no version specifier at all in `pyproject.toml` (D-08: they are pinned
only by `uv.lock`), and `uv.lock` actually resolves a concrete version for
each of the unconstrained transitives this project depends on, especially
`langgraph-prebuilt` -- the one package RESEARCH.md's Pitfall 1 documents as
having broken real callers upstream on an unpinned re-resolve
(`langchain-ai/langgraph#6363`). What this test deliberately does NOT check:
whether a pinned version is still the *latest* one, or whether it still
exists upstream -- re-resolving is a `run.sh install`-time/planning-time
concern, not something this test asserts on every run.

This suite is model-free: a static `tomllib` parse of two already-committed
files, no network, no `rclpy`, no LLM key, no GPU (FOUND-06).
"""

from __future__ import annotations

import tomllib
from pathlib import Path

import pytest

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
PYPROJECT_PATH = EXPERIMENT_DIR / "pyproject.toml"
UV_LOCK_PATH = EXPERIMENT_DIR / "uv.lock"

# The two direct RAI dependencies this milestone locked, and the exact
# version each was verified against (see 01-01-SUMMARY.md's package
# legitimacy audit). A range (">=", "<", "~=", ...) or a different pin is a
# deliberate re-lock this test must catch, not silently accept.
DIRECT_PINS = {
    "rai-core": "2.12.0",
    "rai-whoami": "0.0.5",
}

# Distribution-name prefixes whose *direct* pyproject.toml entry, per D-08,
# must carry no version specifier at all -- these are RAI's own transitives,
# pinned only in uv.lock. A prefix (not an exact name) because the
# ecosystem is many packages: langchain, langchain-core, langchain-aws, ...
UNCONSTRAINED_PREFIXES = ("langchain", "langgraph")

# The single package RESEARCH.md's Pitfall 1 names as having broken real
# callers upstream on an unpinned re-resolve -- its presence in uv.lock is
# the literal guarantee this test exists to enforce.
CRITICAL_TRANSITIVE = "langgraph-prebuilt"

# The rest of the unconstrained set RESEARCH.md's Pitfall 1 names. At least
# four of these five (plus the critical one above) must have a resolved
# entry in uv.lock -- not necessarily all five, since a resolver may drop
# one that becomes an optional extra on a future re-lock.
OTHER_UNCONSTRAINED_TRANSITIVES = (
    "langchain-aws",
    "langchain-openai",
    "langchain-ollama",
    "langchain-google-genai",
    "langchain-community",
)
MIN_OTHER_TRANSITIVES_RESOLVED = 4


def _load_toml(path: Path) -> dict:
    assert path.is_file(), f"missing {path}"
    text = path.read_bytes()
    assert text, f"{path} is empty"
    try:
        return tomllib.loads(text.decode("utf-8"))
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"{path} does not parse as TOML: {exc}") from exc


def _direct_dependency_specs() -> dict[str, str]:
    """Map dependency name -> its full requirement string, from pyproject.toml."""
    data = _load_toml(PYPROJECT_PATH)
    deps = data.get("project", {}).get("dependencies", [])
    specs = {}
    for entry in deps:
        # A PEP 508 requirement string, e.g. "rai-core==2.12.0" or
        # "langchain-core>=1.0". Split on the first version-comparison
        # operator to get the bare name.
        for op in ("==", ">=", "<=", "~=", "!=", ">", "<"):
            if op in entry:
                name = entry.split(op, 1)[0].strip()
                specs[name] = entry.strip()
                break
        else:
            specs[entry.strip()] = entry.strip()
    return specs


def _lock_package_versions() -> dict[str, str]:
    data = _load_toml(UV_LOCK_PATH)
    packages = data.get("package", [])
    assert packages, f"{UV_LOCK_PATH} has no [[package]] entries -- it must not be empty"
    return {p["name"]: p.get("version", "") for p in packages if "name" in p}


@pytest.mark.parametrize("name,expected_version", sorted(DIRECT_PINS.items()))
def test_direct_dependency_is_pinned_to_exact_version(name: str, expected_version: str):
    specs = _direct_dependency_specs()
    assert name in specs, f"{PYPROJECT_PATH.name} declares no direct dependency on {name!r}"
    spec = specs[name]
    assert spec == f"{name}=={expected_version}", (
        f"{PYPROJECT_PATH.name}: {name!r} must be pinned with '==' to exactly "
        f"{expected_version!r}, found {spec!r}"
    )


def test_no_direct_pyproject_entry_pins_a_langchain_or_langgraph_package():
    """D-08: RAI's own transitives stay unpinned in pyproject.toml -- uv.lock
    is their only real pin, and this must be a deliberate choice, not an
    accident a future edit silently undoes.
    """
    specs = _direct_dependency_specs()
    offenders = [
        name
        for name in specs
        if name.startswith(UNCONSTRAINED_PREFIXES) and name not in DIRECT_PINS
    ]
    assert not offenders, (
        f"{PYPROJECT_PATH.name} pins a LangChain/LangGraph transitive "
        f"directly (D-08 says these stay unpinned here, pinned only in "
        f"uv.lock): {offenders}"
    )


def test_lockfile_exists_non_empty_and_parses():
    versions = _lock_package_versions()
    assert versions, f"{UV_LOCK_PATH} parsed but resolved no packages"


def test_lockfile_pins_the_critical_transitive():
    versions = _lock_package_versions()
    assert CRITICAL_TRANSITIVE in versions, (
        f"{UV_LOCK_PATH} has no resolved entry for {CRITICAL_TRANSITIVE!r} -- "
        "this is the single package RESEARCH.md's Pitfall 1 documents as "
        "having broken real callers upstream on an unpinned re-resolve; the "
        "lockfile is its only real pin"
    )
    assert versions[CRITICAL_TRANSITIVE], (
        f"{UV_LOCK_PATH}: {CRITICAL_TRANSITIVE!r} entry has no version"
    )


def test_lockfile_pins_most_of_the_other_unconstrained_transitives():
    versions = _lock_package_versions()
    resolved = [
        name
        for name in OTHER_UNCONSTRAINED_TRANSITIVES
        if versions.get(name)
    ]
    assert len(resolved) >= MIN_OTHER_TRANSITIVES_RESOLVED, (
        f"{UV_LOCK_PATH} resolved only {resolved} of "
        f"{OTHER_UNCONSTRAINED_TRANSITIVES} -- expected at least "
        f"{MIN_OTHER_TRANSITIVES_RESOLVED}"
    )


def test_missing_lockfile_is_rejected(tmp_path):
    """Fail-first control: an absent lockfile must be caught, not silently
    treated as 'nothing to check'.
    """
    missing = tmp_path / "does-not-exist.lock"
    with pytest.raises(AssertionError, match="missing"):
        _load_toml(missing)


def test_empty_lockfile_is_rejected(tmp_path):
    """Fail-first control: an empty (zero-byte) lockfile must be caught."""
    empty = tmp_path / "empty.lock"
    empty.write_bytes(b"")
    with pytest.raises(AssertionError, match="is empty"):
        _load_toml(empty)


def test_unparseable_lockfile_is_rejected(tmp_path):
    """Fail-first control: a lockfile that isn't valid TOML must be caught,
    not silently treated as 'zero packages, nothing to check'.
    """
    garbage = tmp_path / "garbage.lock"
    garbage.write_text("this is not [ valid toml")
    with pytest.raises(ValueError, match="does not parse as TOML"):
        _load_toml(garbage)
