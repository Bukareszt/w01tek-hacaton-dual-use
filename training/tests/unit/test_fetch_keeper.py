"""fetch_keeper's pure parts: reference parsing, step naming, the layout.

The network and env-building halves are exercised by hand (a keeper repo
is private and an env is a model); everything a wrong path or a wrong
step name would break is here.
"""

import pytest

from wojtek_rl import paths
from wojtek_rl.fetch_keeper import (
    checkpoint_step,
    parse_repo_ref,
    run_dir_layout,
)


def test_bare_name_takes_the_organization_and_defaults_to_main():
    assert parse_repo_ref("wojtek-quiet-locomotion", "org") == (
        "org/wojtek-quiet-locomotion", "main",
    )


def test_full_id_keeps_its_org_and_revision():
    assert parse_repo_ref("other/x@553795b", "org") == ("other/x", "553795b")


def test_bare_name_without_an_organization_is_refused_locally():
    with pytest.raises(ValueError, match="HF_ORGANIZATION"):
        parse_repo_ref("wojtek-quiet-locomotion", "")


def test_checkpoint_step_is_the_contracts_last_path_component():
    meta = {"checkpoint": "/somewhere/runs/x/checkpoints/001002700800"}
    assert checkpoint_step(meta) == "001002700800"


def test_checkpoint_step_falls_back_to_zero_when_the_contract_has_none():
    assert checkpoint_step({}) == "000000000000"
    assert checkpoint_step({"checkpoint": "/not/a/step"}) == "000000000000"


def test_layout_matches_what_train_and_the_eval_tools_expect():
    lay = run_dir_layout("wojtek_x", "001002700800")
    run_dir = paths.PROJECT_DIR / "runs" / "wojtek_x"
    assert lay["run_dir"] == run_dir
    assert lay["checkpoint"] == run_dir / "checkpoints" / "001002700800"
    assert lay["deploy"] == run_dir / "deploy"
    assert lay["run_json"] == run_dir / "run.json"
