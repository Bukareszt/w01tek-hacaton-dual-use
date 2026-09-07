"""Working-directory-independent access to this experiment's config.toml.

RAI's own `load_config(config_path=None)` opens a bare relative
`"config.toml"` against whatever the process's current working directory
happens to be at call time -- and this experiment's container defaults to
`/ros2_ws`, not `experiments/wojtek_rai_v1/` (see 01-RESEARCH.md Pitfall 5).
`config_path()` anchors on this module's own file location instead
(`Path(__file__).resolve().parent.parent`), so any code in this and later
phases that needs the config goes through this function, or passes its
result explicitly to RAI's own loader, rather than relying on cwd.

`load_experiment_config()` parses that path with the standard library's
`tomllib` and returns the mapping -- no RAI import, no network, no
environment access, so this module stays importable and testable with
nothing installed but the standard library (FOUND-06).

`required_env_vars()` returns names only, never values: it maps a vendor
name from `config.toml`'s `[vendor]` table to the environment variable
names that vendor's LangChain integration reads credentials from (see
01-RESEARCH.md Pitfall 6 and Code Examples for the exact names, verified
against RAI's `model_initialization.py`). Every name it can return must
stay declared in the repo-root `.env.example`
(tests/test_config_loading.py cross-checks this) so the template can never
silently fall behind this mapping.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

# Vendor name (config.toml's [vendor] simple_model/complex_model/
# embeddings_model value) -> the environment variable names that vendor's
# LangChain integration reads credentials from. Ollama needs none (local,
# no auth). Extend this mapping, and .env.example, together when a new
# vendor is added (HRI-03).
_VENDOR_ENV_VARS: dict[str, tuple[str, ...]] = {
    "openai": ("OPENAI_API_KEY",),
    "google": ("GOOGLE_API_KEY",),
    "aws": ("AWS_ACCESS_KEY_ID", "AWS_SECRET_ACCESS_KEY", "AWS_SESSION_TOKEN"),
    "ollama": (),
}


def config_path() -> Path:
    """Absolute path to this experiment's committed config.toml.

    Anchored on this module's own location, not the process working
    directory -- see the module docstring for why that distinction matters
    here.
    """
    return (Path(__file__).resolve().parent.parent / "config.toml").resolve()


def load_experiment_config() -> dict:
    """Parse and return this experiment's config.toml as a mapping.

    Raises `FileNotFoundError` naming the absolute path it tried when the
    file is missing, rather than a bare `tomllib`/`open` error with only a
    relative name.
    """
    path = config_path()
    try:
        with path.open("rb") as f:
            return tomllib.load(f)
    except FileNotFoundError:
        raise FileNotFoundError(
            f"RAI experiment config.toml not found at {path}"
        ) from None


def required_env_vars(vendor: str) -> tuple[str, ...]:
    """Environment variable names the given vendor needs credentials from.

    Returns names only -- never reads, logs, or returns a value. Raises
    `ValueError` naming the offending vendor string when it is not one of
    the four known vendors.
    """
    try:
        return _VENDOR_ENV_VARS[vendor]
    except KeyError:
        raise ValueError(f"unknown RAI vendor: {vendor!r}") from None
