"""The suite's own model-free contract (FOUND-06): this test suite needs no
model key, no ROS runtime and no GPU.

What this repository cares about: no test file imports a ROS 2 client
library, a RAI package, or a model vendor SDK at *module* scope -- a
module-scope import runs the moment pytest collects the file, before any
test decides whether it actually needs that dependency, and would make
`run.sh test` fail to even start without a live ROS daemon, an LLM key or a
GPU driver. A *function*-local import (inside a test body) is fine and is
the convention this experiment's own `wojtek_rai/topics.py` and
`wojtek_rai/config.py` already follow -- it only runs when that specific
test actually executes. Also asserts the suite itself is non-vacuous
(a below-threshold collected file count is an error, not a clean run) and
that no test is marked `skip` unconditionally, which would let this guard
-- or any other -- silently stop running while `run.sh test` still reports
success.

This suite is model-free: `ast` (parse only, never execute the files it
scans), the standard library, no network, no `rclpy`, no LLM key, no GPU.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent

# Distribution/module prefixes that would make collecting -- not even
# running -- a test require a live ROS daemon, ping a vendor API, or need a
# GPU driver present. Matched against the *first* dotted component of an
# import name, so "rai.communication.ros2" and "rai" both match "rai", but
# an unrelated name that merely starts with the same letters (there are
# none in this suite today) would not, because we compare whole components.
FORBIDDEN_MODULE_SCOPE_PREFIXES = (
    "rclpy",  # ROS 2 client library -- needs a live ROS 2 daemon/context
    "rai",  # RAI itself (rai.*) -- rai-core's own import chain pulls in
    # cv_bridge (RESEARCH.md Pitfall 2) and, transitively, LangChain
    "rai_whoami",
    "cv_bridge",  # needs ROS 2's vision_opencv stack
    # Model vendor SDKs -- would attempt real vendor calls, or at minimum
    # import chains that assume network/model availability.
    "openai",
    "anthropic",
    "boto3",
    "langchain",
    "langgraph",
    "ollama",
)

# This plan (01-04) creates 4 new guard test files matching test_*.py;
# combined with the 5 pre-existing ones from plans 01-01..01-03
# (test_topics_module, test_repos_pin, test_build_target,
# test_config_loading, test_no_secrets_in_config -- conftest.py itself
# does not match this glob), never fewer than this many files should ever
# be collected. A below-threshold count means test discovery silently
# broke, not that the suite genuinely shrank.
MIN_EXPECTED_TEST_FILES = 9


def _test_files() -> list[Path]:
    return sorted(TESTS_DIR.glob("test_*.py"))


def _module_scope_import_names(path: Path) -> list[str]:
    """Every dotted import name appearing at module scope (not nested
    inside a function/method/class body) in `path`.
    """
    tree = ast.parse(path.read_text(), filename=str(path))
    names = []
    for node in tree.body:  # module.body: only top-level statements
        if isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.append(node.module)
    return names


def test_suite_is_non_vacuous():
    files = _test_files()
    assert len(files) >= MIN_EXPECTED_TEST_FILES, (
        f"only {len(files)} test files collected in {TESTS_DIR} -- expected "
        f"at least {MIN_EXPECTED_TEST_FILES}; test discovery may be broken"
    )


@pytest.mark.parametrize("path", _test_files(), ids=lambda p: p.name)
def test_no_module_scope_import_of_a_ros_rai_or_vendor_module(path: Path):
    names = _module_scope_import_names(path)
    offenders = [
        name
        for name in names
        if name.split(".")[0] in FORBIDDEN_MODULE_SCOPE_PREFIXES
    ]
    assert not offenders, (
        f"{path.name} imports {offenders} at module scope -- collecting "
        "this file would require a live ROS 2 daemon, a model vendor SDK, "
        "or a GPU driver before any test decides whether it actually needs "
        "one; move the import inside the function that uses it"
    )


def test_no_test_is_marked_skip_unconditionally():
    """A test decorated with a bare `@pytest.mark.skip` (no condition) never
    runs again, silently -- `run.sh test` would keep reporting success even
    though that guard stopped guarding anything. `pytest.mark.skipif` (a
    conditional skip) is not checked here: it can legitimately turn off in
    one environment and on in another, which is a different, deliberate
    contract.
    """
    offenders = []
    for path in _test_files():
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            for decorator in node.decorator_list:
                dotted = ast.unparse(decorator) if hasattr(ast, "unparse") else ""
                # Match "pytest.mark.skip" or "mark.skip" but not
                # "...skipif(...)" -- skip() with no call, or a bare
                # attribute access used as a decorator.
                if dotted in {"pytest.mark.skip", "mark.skip"} or dotted.startswith(
                    ("pytest.mark.skip(", "mark.skip(")
                ):
                    offenders.append(f"{path.name}::{node.name}")
    assert not offenders, (
        f"test(s) marked unconditionally skipped: {offenders} -- a skipped "
        "guard test stops guarding while run.sh test keeps reporting success"
    )


def test_forbidden_prefix_detection_is_exercised_by_a_synthetic_violation(tmp_path):
    """Fail-first control: a synthetic file with a real module-scope
    forbidden import must be caught by the same detection this test file
    uses on the real suite -- built at runtime, never written as a literal
    top-level import in this source file (which would itself trip a linter
    or, worse, an accidental real dependency).
    """
    probe = tmp_path / "test_synthetic_probe.py"
    probe.write_text("import " + "rclpy" + "\n\n\ndef test_x():\n    pass\n")
    names = _module_scope_import_names(probe)
    offenders = [n for n in names if n.split(".")[0] in FORBIDDEN_MODULE_SCOPE_PREFIXES]
    assert offenders, "detection failed to catch a synthetic module-scope ROS import"


def test_function_local_import_is_not_flagged(tmp_path):
    """Positive control: the same forbidden name, imported inside a
    function body (this suite's own convention), must NOT be flagged --
    only a *module-scope* import is the violation.
    """
    probe = tmp_path / "test_synthetic_local_import.py"
    probe.write_text(
        "def test_x():\n    import " + "rclpy" + "\n    assert rclpy\n"
    )
    names = _module_scope_import_names(probe)
    offenders = [n for n in names if n.split(".")[0] in FORBIDDEN_MODULE_SCOPE_PREFIXES]
    assert not offenders, "a function-local import was wrongly flagged as module-scope"
