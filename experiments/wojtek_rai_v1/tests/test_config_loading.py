"""Model-free coverage of `wojtek_rai.config`.

Covers the behaviours this module promises (see the plan's <behavior>
block): `config_path()` is cwd-independent, `load_experiment_config()`
returns the same mapping regardless of the caller's working directory and
fails loudly (naming the path it tried) when the file is missing, and
`required_env_vars()` maps each of the four vendors this milestone's
`config.toml` declares to the exact environment variable names that vendor
needs -- names that must always be present in the repo-root `.env.example`
so the template can never silently fall behind the code.

This suite needs no ROS 2, no LLM key, and no GPU (FOUND-06): it never
imports `rai` or `rclpy`, only the standard library plus this module.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from wojtek_rai import config

REPO_ROOT = Path(__file__).resolve().parents[3]
ENV_EXAMPLE = REPO_ROOT / ".env.example"


def test_config_path_is_absolute_and_ends_at_the_experiment_config_toml():
    path = config.config_path()
    assert path.is_absolute()
    assert path.name == "config.toml"
    assert path.parent.name == "wojtek_rai_v1"


def test_config_path_is_independent_of_the_working_directory(
    monkeypatch, tmp_path
):
    from_here = config.config_path()
    monkeypatch.chdir(tmp_path)
    from_elsewhere = config.config_path()
    assert from_elsewhere == from_here


def test_load_experiment_config_is_independent_of_the_working_directory(
    monkeypatch, tmp_path
):
    from_here = config.load_experiment_config()
    monkeypatch.chdir(tmp_path)
    from_elsewhere = config.load_experiment_config()
    assert from_elsewhere == from_here
    # Sanity: this is actually the committed config, not an empty mapping.
    assert from_here["vendor"]["simple_model"] == "openai"


def test_load_experiment_config_raises_file_not_found_naming_the_path(
    monkeypatch, tmp_path
):
    missing = tmp_path / "config.toml"
    monkeypatch.setattr(config, "config_path", lambda: missing)
    with pytest.raises(FileNotFoundError, match=re.escape(str(missing))):
        config.load_experiment_config()


@pytest.mark.parametrize(
    "vendor,expected",
    [
        ("openai", ("OPENAI_API_KEY",)),
        ("google", ("GOOGLE_API_KEY",)),
        ("aws", ("AWS_ACCESS_KEY_ID", "AWS_SECRET_ACCESS_KEY", "AWS_SESSION_TOKEN")),
        ("ollama", ()),
    ],
)
def test_required_env_vars_maps_each_known_vendor_exactly(vendor, expected):
    assert config.required_env_vars(vendor) == expected


def test_required_env_vars_raises_value_error_naming_the_unknown_vendor():
    with pytest.raises(ValueError, match="not-a-real-vendor"):
        config.required_env_vars("not-a-real-vendor")


def test_every_required_env_var_name_is_declared_in_root_env_example():
    text = ENV_EXAMPLE.read_text()
    declared = set(re.findall(r"^([A-Z][A-Z0-9_]*)=", text, re.M))
    all_names = {
        name
        for vendor in ("openai", "google", "aws", "ollama")
        for name in config.required_env_vars(vendor)
    }
    missing = all_names - declared
    assert not missing, (
        f"{ENV_EXAMPLE} is missing placeholder(s) for: {sorted(missing)}"
    )
