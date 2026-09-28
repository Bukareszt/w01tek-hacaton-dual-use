"""Materialize a published keeper as a local run directory.

A keeper's Hugging Face repo carries its final orbax checkpoint (under
`checkpoint/`) and the deploy pair, but eval, report, export and
`restore=` all read a RUN DIRECTORY: `runs/<name>/run.json` next to
`runs/<name>/checkpoints/<step>/`. This tool builds that directory:

    ./run.sh fetch-keeper --repo wojtek-quiet-locomotion@main \\
        --experiment flat_quiet_baseacc --run-name wojtek_flat_quiet_baseacc_s0

  runs/<run-name>/checkpoints/<step>/   the repo's checkpoint/ tree, where
                                        <step> is the step the keeper's
                                        contract names
  runs/<run-name>/deploy/               policy.npz + policy_meta.json (the
                                        export marker courses_battery.sh
                                        looks for)
  runs/<run-name>/run.json              the record train.py would have
                                        written

When the repo ships its own run.json (the stiff/springy keepers do), that
file is the record, with checkpoint_dir repointed here. When it does not
(the quiet keepers), the record is SYNTHESIZED from a Hydra preset: the
preset plus any overrides resolve exactly as `train` would resolve them,
the env is built once on CPU for its effective config and gains, and
run.json says so (`source.reconstructed_from`). The preset must be the
recipe the keeper was trained with -- a record preset such as
flat_quiet_v10 -- because eval tools rebuild the env from it; a wrong
obs layout is caught at policy load, a wrong reward scale is not.

A bare repo name resolves under HF_ORGANIZATION (environment or the
repo-root .env); with neither, the token's only organization.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

from wojtek_rl import paths

CHECKPOINT_PREFIX = "checkpoint/"
DEPLOY_FILES = ("policy.npz", "policy_meta.json")


def parse_repo_ref(ref: str, organization: str = "") -> tuple[str, str]:
    """(repo_id, revision) from `[org/]name[@revision]`.

    A bare name takes `organization`; an empty organization with a bare
    name is an error here, not a network call.
    """
    ref = ref.strip()
    if not ref:
        raise ValueError("empty repo reference")
    repo, _, revision = ref.partition("@")
    if "/" not in repo:
        if not organization:
            raise ValueError(
                f"{ref!r} has no organization and HF_ORGANIZATION is not set"
            )
        repo = f"{organization}/{repo}"
    return repo, revision or "main"


def checkpoint_step(meta: dict) -> str:
    """The step directory name the keeper's contract points at.

    The contract records the training host's absolute checkpoint path; its
    last component is the zero-padded step, which is what brax names the
    directory. A contract without one gets step 0.
    """
    ckpt = str(meta.get("checkpoint") or "").rstrip("/")
    name = ckpt.rsplit("/", 1)[-1] if ckpt else ""
    return name if name.isdigit() else "0" * 12


def run_dir_layout(run_name: str, step: str) -> dict[str, Path]:
    """Where the pieces land, relative to the training project."""
    run_dir = paths.PROJECT_DIR / "runs" / run_name
    return {
        "run_dir": run_dir,
        "checkpoint": run_dir / "checkpoints" / step,
        "deploy": run_dir / "deploy",
        "run_json": run_dir / "run.json",
    }


def synthesize_record(run_name: str, experiment: str, overrides: list[str],
                      checkpoint_dir: Path, source: dict) -> dict:
    """The run.json train.py would have written for this recipe.

    Resolves the Hydra config the way `train` does (preset + overrides on
    the same config tree), builds the env once on the jax backend with a
    single world for its effective config and the actuator gains, and
    records the provenance under `source`. The stored env_config.sim
    therefore says backend=jax, num_envs=1 where a trained run records
    its training batch; the eval tools override sim themselves, so only
    `eval` on a GPU host would notice (it steps the recorded backend).
    """
    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf

    from wojtek_rl.registry import make_env
    from wojtek_rl.train import _apply_ppo_overrides, build_ppo_params

    conf_dir = paths.PROJECT_DIR / "wojtek_rl" / "conf"
    with initialize_config_dir(config_dir=str(conf_dir), version_base=None):
        cfg = compose(
            config_name="config",
            overrides=[f"+experiment={experiment}", *overrides],
        )

    task = cfg.task.name
    env_overrides = OmegaConf.to_container(cfg.task.env, resolve=True) or {}
    ppo_params = build_ppo_params({}, smoke=False)
    _apply_ppo_overrides(
        ppo_params.network_factory,
        OmegaConf.to_container(cfg.network, resolve=True) or {},
    )
    _apply_ppo_overrides(
        ppo_params, OmegaConf.to_container(cfg.task.ppo, resolve=True) or {}
    )
    _apply_ppo_overrides(
        ppo_params, OmegaConf.to_container(cfg.ppo, resolve=True) or {}
    )
    # One world on the jax backend: the record needs the resolved config
    # and the gains, not a training-sized contact buffer or a GPU.
    sim = dict(env_overrides.get("sim") or {})
    sim.update(backend="jax", num_envs=1)
    env_overrides["sim"] = sim
    env = make_env(task, env_overrides)

    return {
        "run_name": run_name,
        "task": task,
        "status": "published",
        "num_timesteps": int(ppo_params.num_timesteps),
        "final_reward": None,
        "checkpoint_dir": str(checkpoint_dir.parent),
        "env_config": env._config.to_dict(),
        "ppo_config": ppo_params.to_dict(),
        "hydra_config": OmegaConf.to_container(cfg, resolve=True),
        "kp": float(env.mj_model.actuator_gainprm[0, 0]),
        "kd": float(-env.mj_model.actuator_biasprm[0, 2]),
        "source": {**source, "reconstructed_from": experiment,
                   "overrides": list(overrides)},
    }


def _organization() -> str:
    if paths.HF_ORGANIZATION:
        return paths.HF_ORGANIZATION
    from huggingface_hub import whoami

    orgs = [o.get("name") for o in whoami().get("orgs", [])]
    return orgs[0] if len(orgs) == 1 else ""


def fetch(ref: str, run_name: str | None, experiment: str | None,
          overrides: list[str], force: bool) -> Path:
    from huggingface_hub import hf_hub_download, list_repo_files

    repo_id, revision = parse_repo_ref(ref, _organization())
    files = list_repo_files(repo_id, revision=revision)
    ckpt_files = [f for f in files if f.startswith(CHECKPOINT_PREFIX)]
    if not ckpt_files:
        raise SystemExit(f"{repo_id}@{revision} ships no checkpoint/ tree")

    if "policy_meta.json" not in files:
        raise SystemExit(
            f"{repo_id}@{revision} ships no policy_meta.json; the contract "
            "names the checkpoint step and the run, so it is required"
        )
    meta = json.loads(
        Path(hf_hub_download(repo_id, "policy_meta.json", revision=revision))
        .read_text()
    )
    run_name = run_name or meta.get("run_name")
    if not run_name:
        raise SystemExit("--run-name is required: the contract names no run")
    step = checkpoint_step(meta)
    layout = run_dir_layout(run_name, step)
    if layout["run_json"].exists() and not force:
        raise SystemExit(
            f"{layout['run_dir']} already exists; pass --force to rebuild it"
        )
    if "run.json" in files and (experiment or overrides):
        raise SystemExit(
            f"{repo_id}@{revision} ships its own run.json; --experiment and "
            "overrides only apply to a synthesized record"
        )

    # A rebuild replaces the whole run dir. Every reader picks the highest
    # numbered step under checkpoints/, so a step left over from an earlier
    # fetch would silently win over the one this fetch brings.
    if force:
        shutil.rmtree(layout["checkpoint"].parent, ignore_errors=True)
        shutil.rmtree(layout["deploy"], ignore_errors=True)
    layout["checkpoint"].mkdir(parents=True, exist_ok=True)
    for f in ckpt_files:
        src = Path(hf_hub_download(repo_id, f, revision=revision))
        dst = layout["checkpoint"] / f[len(CHECKPOINT_PREFIX):]
        dst.parent.mkdir(parents=True, exist_ok=True)
        # The download cache hands back symlinks; the run dir gets real
        # files so it rsyncs and restores like a trained run.
        shutil.copyfile(src, dst)
    layout["deploy"].mkdir(parents=True, exist_ok=True)
    for f in DEPLOY_FILES:
        if f in files:
            shutil.copyfile(
                Path(hf_hub_download(repo_id, f, revision=revision)),
                layout["deploy"] / f,
            )

    source = {"repo": repo_id, "revision": revision, "contract_step": step}
    if "run.json" in files:
        record = json.loads(
            Path(hf_hub_download(repo_id, "run.json", revision=revision))
            .read_text()
        )
        record["checkpoint_dir"] = str(layout["checkpoint"].parent)
        # The directory name is what every reader labels its output with.
        record["run_name"] = run_name
        record["source"] = source
    else:
        if not experiment:
            raise SystemExit(
                f"{repo_id}@{revision} ships no run.json; pass --experiment "
                "<record preset> so the record can be synthesized"
            )
        record = synthesize_record(
            run_name, experiment, overrides, layout["checkpoint"], source
        )
    layout["run_json"].write_text(json.dumps(record, indent=2, default=str))
    return layout["run_dir"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--repo", required=True,
                    help="[org/]name[@revision]; bare name under HF_ORGANIZATION")
    ap.add_argument("--run-name", default=None,
                    help="runs/<run-name>; default: the contract's run_name")
    ap.add_argument("--experiment", default=None,
                    help="record preset to synthesize run.json from when the "
                         "repo ships none (e.g. flat_quiet_baseacc)")
    ap.add_argument("--force", action="store_true",
                    help="rebuild an existing run dir")
    ap.add_argument("overrides", nargs="*",
                    help="extra Hydra overrides for the synthesized record")
    args = ap.parse_args(argv)
    run_dir = fetch(
        args.repo, args.run_name, args.experiment, args.overrides, args.force
    )
    rel = run_dir.relative_to(paths.PROJECT_DIR)
    print(f"keeper materialized at {rel}")
    print(f"  courses:  ./training/run.sh courses --run {rel}")
    steps = sorted(p.name for p in (run_dir / "checkpoints").iterdir())
    print(f"  restore:  restore={rel}/checkpoints/{steps[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
