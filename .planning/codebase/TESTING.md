# Testing Patterns

**Analysis Date:** 2026-09-05

## Test Framework

**Runner:**
- `pytest` >=8.0.0 (declared in `training/pyproject.toml` dev dependencies)
- Config: No `pytest.ini` or `setup.cfg` test config; relies on pytest defaults with `-q` (quiet) flag

**Assertion Library:**
- pytest's built-in `assert` statements
- `numpy.testing` module for array comparisons (e.g., `np.testing.assert_allclose`, `np.testing.assert_array_equal`)
- `pytest.approx()` for floating-point tolerance testing

**Run Commands:**

From repository root (`/home/bukareszt/Downloads/robodog/w01-tek/`):

```bash
./training/run.sh test              # Run fast unit tests (tests/unit/)
./training/run.sh test-slow         # Run slow integration tests (tests/integration/)
./training/run.sh test-all          # Run all tests (both suites)
```

Standalone pytest (if venv sourced):
```bash
pytest tests/unit -q
pytest tests/integration -q
JAX_COMPILATION_CACHE_DIR=.jax_cache pytest tests/integration -q
```

## Test File Organization

**Location:**
- Unit tests: `training/tests/unit/` — model-free, model-agnostic
- Integration tests: `training/tests/integration/` — environment instantiation, MJX compilation
- Hard split enforced by guard test: `tests/unit/test_job_scripts.py` validates scripts carry no site-specific values
- ROS tests: `ros/src/[package]/test/` (e.g., `ros/src/wojtek_policy/test/`)
- Experiment tests: `experiments/[exp]/tests/` (e.g., `experiments/autonomous_architecture_ros2_v1/tests/`)

**Naming:**
- `test_*.py` — standard test files (e.g., `test_battery.py`, `test_env.py`, `test_policy.py`)
- `*_test.py` — alternative test file naming (less common in this codebase)
- Test function names: `test_<behavior_description>` (e.g., `test_obs_shapes`, `test_pd_hold_keeps_robot_up`)

**Structure:**
```
training/tests/
├── unit/                          # Fast, no MJX instantiation
│   ├── test_battery.py
│   ├── test_job_scripts.py        # Hygiene guard tests
│   └── test_*.py
├── integration/                   # Slow, real MJX/JAX
│   ├── test_env.py
│   ├── test_locomotion.py
│   └── test_*.py
└── capture_goldens.py             # Shared utility

ros/src/[package]/test/
├── test_*.py

experiments/[exp]/tests/
└── test_*.py
```

## Test Structure

**Suite Organization:**
```python
# Unit test example (test_battery.py)
import numpy as np
import pytest

from wojtek_rl.battery import vibration_index, diag_corr, lateral_corr

def test_battery_scenarios_step_counts():
    scenarios = battery_scenarios()
    assert set(scenarios.keys()) == {...}
    assert scenarios["stand_to_trot_ramp"][1] == 750

def test_diag_corr_trot_gait_low_lateral_high():
    contacts = np.array([[1, 0, 1, 0], ...], dtype=bool)
    assert diag_corr(contacts) > 0.99
    assert lateral_corr(contacts) < -0.99

# Integration test example (test_env.py)
import jax
import jax.numpy as jp
import pytest

from wojtek_rl import env as wojtek_env

@pytest.fixture(scope="module")
def env():
    return wojtek_env.WojtekJoystick()

@pytest.fixture(scope="module")
def reset_state(env):
    return jax.jit(env.reset)(jax.random.PRNGKey(0))

def test_obs_shapes(env, reset_state):
    assert reset_state.obs["state"].shape == (wojtek_env.OBS_SIZE,)
    assert reset_state.obs["privileged_state"].shape == (wojtek_env.PRIVILEGED_SIZE,)

def test_step_produces_finite_reward(env, reset_state):
    state = jax.jit(env.step)(reset_state, jp.zeros(12))
    assert np.isfinite(float(state.reward))
    for v in state.metrics.values():
        assert np.isfinite(float(v))
```

**Patterns:**

- **Arrange-Act-Assert** (AAA):
  ```python
  def test_action_filter_is_ema(tmp_path):
      # Arrange
      b = np.full(12, 0.5, np.float32)
      pol = make_policy(tmp_path, bias12=b, meta_updates={"action_filter": 0.8})
      
      # Act
      a = np.tanh(b)
      args = (np.zeros(3), [0, 0, -1.0], pol.home_ctrl, np.zeros(12), [0.2, 0, 0])
      t1 = pol.step(*args)
      
      # Assert
      scale = np.array(META["action_scale"])
      anchor = np.array(META["anchor_ctrl"])
      assert np.allclose(t1, np.clip(anchor + 0.2 * a * scale, lo, hi), atol=1e-6)
  ```

- **Setup/Teardown:** Fixtures with scope (`scope="module"` for expensive setups like environment init)
  ```python
  @pytest.fixture(scope="module")
  def env():
      return wojtek_env.WojtekJoystick()
  ```

- **Parametrization:**
  ```python
  @pytest.mark.parametrize("name", sorted(tricks.TRICKS))
  def test_clip_starts_and_ends_home(name):
      assert np.allclose(tricks.sample(name, 0.0), HOME_CTRL, atol=1e-9)
  ```

- **Assertion with tolerance:**
  ```python
  def _close(a, b, tol=1e-5):
      return abs(a - b) < tol
  
  assert _close(float(cmd(100)[0]), 0.4)
  assert np.allclose(obs[0:12], q - policy.home_ctrl, atol=1e-6)
  assert np.all(np.isfinite(np.asarray(state.info["gyro_vib"])))
  ```

## Mocking

**Framework:** pytest's `monkeypatch` fixture (built-in)

**Patterns:**
```python
def test_fetch_materializes_real_files(tmp_path, monkeypatch):
    # Patch environment variable
    monkeypatch.setenv("WOJTEK_POLICY_STORE", str(tmp_path))
    
    # Patch module function
    def no_network(*args, **kwargs):
        raise AssertionError("network attempted")
    
    monkeypatch.setattr(policy_source, "_fetch_into_store", no_network)
    
    # Patch sys.modules to inject fake dependencies
    monkeypatch.setitem(
        sys.modules, "huggingface_hub",
        types.SimpleNamespace(hf_hub_download=fake_download)
    )
```

**What to Mock:**
- Network calls (e.g., HuggingFace downloads)
- External service calls (policy store fetches)
- Environment configuration (env vars via `monkeypatch.setenv`)
- Module functions/classes that are slow or external

**What NOT to Mock:**
- Core simulation logic (env step, reward computation)
- Policy forward passes
- File I/O (use `tmp_path` temporary directories instead)
- Math/numpy operations
- JAX compilation

## Fixtures and Factories

**Test Data:**

Custom factory functions create synthetic policies for testing:

```python
def make_policy(tmp_path, bias12=None, meta_updates=None, clamp_knee=False):
    """Synthetic zero-kernel policy: action = tanh(bias) for any obs."""
    meta = dict(META)
    meta.update(meta_updates or {})
    obs_size = meta["obs_size"]
    bias = np.zeros(24, np.float32)
    if bias12 is not None:
        bias[:12] = bias12
    np.savez(
        tmp_path / "policy.npz",
        norm_mean=np.zeros(obs_size, np.float32),
        norm_std=np.ones(obs_size, np.float32),
        hidden_0_kernel=np.zeros((obs_size, 24), np.float32),
        hidden_0_bias=bias,
    )
    (tmp_path / "policy_meta.json").write_text(json.dumps(meta))
    return WojtekPolicy(tmp_path / "policy.npz", clamp_knee=clamp_knee)

@pytest.fixture
def policy(tmp_path):
    return make_policy(tmp_path, bias12=np.linspace(-0.4, 0.4, 12))
```

**Location:**
- Fixtures defined in test files (no shared `conftest.py` for this repo)
- Factory functions (e.g., `make_policy`, `store_snapshot`) in test module with test files that use them
- `tmp_path` pytest built-in for temporary directories

## Coverage

**Requirements:** 
- Not explicitly stated; no coverage thresholds enforced
- Primary validation: "does the policy work on the robot?" — integration tests validate real behavior
- Guard tests validate hygiene (no secrets) and contract compliance (schema version)

**View Coverage:**
```bash
pytest tests/unit --cov=wojtek_rl
pytest tests/integration --cov=wojtek_rl
```

(If `pytest-cov` installed; not listed in dev dependencies but available)

## Test Types

**Unit Tests:**
- Scope: Pure functions, models, mathematics, no external I/O
- Location: `training/tests/unit/`
- Runtime: ~3 seconds for entire suite
- Features: Guard test (`test_job_scripts.py`) validates job payload scripts
- Examples: battery metrics, policy runtime, joint mappings, gravity calculations
- Pattern: Single function tested in isolation with synthetic inputs

**Integration Tests:**
- Scope: Environment instantiation, MJX compilation, multi-step simulation, reward computation
- Location: `training/tests/integration/`
- Runtime: ~10 minutes (JAX JIT compilation is slow on first run; cached on repeat)
- Features: Real MJX models, environment stepping, policy integration
- Examples: observation shapes, action filters, locomotion tasks, terrain, height scanning
- Pattern: Environment fixture at module scope (expensive), reused across tests
- Environment setup:
  ```bash
  JAX_COMPILATION_CACHE_DIR="${JAX_COMPILATION_CACHE_DIR:-$PWD/.jax_cache}" \
  JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS=0 \
  pytest tests/integration -q
  ```

**E2E Tests:**
- Not formally separated; integration tests serve this role
- End-to-end behavior validated through: training run, checkpoint evaluation, policy export, ROS deployment
- Validation: `./training/run.sh check --gpu --backend warp` validates GPU backend; `./training/run.sh check` validates JAX backend

## Common Patterns

**Async Testing:**
- Not used; simulation is synchronous
- JAX JIT compilation handled via `jax.jit()` wrapper in tests
- JAX compilation caching via environment variable `JAX_COMPILATION_CACHE_DIR`

**Error Testing:**
```python
def test_rejects_wrong_schema_version(tmp_path):
    with pytest.raises(ValueError, match="schema_version"):
        make_policy(tmp_path, meta_updates={"schema_version": None})
    with pytest.raises(ValueError, match="schema_version"):
        make_policy(tmp_path, meta_updates={"schema_version": 1})

def test_live_height_contract_requires_ctrlrange(tmp_path):
    updates = {k: v for k, v in LIVE_HEIGHT_META.items()
               if k not in ("ctrl_low", "ctrl_high")}
    with pytest.raises(ValueError, match="ctrl_low/ctrl_high"):
        make_policy(tmp_path, meta_updates=updates)
```

**Contract Validation Tests:**
- Policy meta schema version checked at load
- Obs layout components validated against `KNOWN_COMPONENTS`
- Action size matched to contract (tau_ff enabled doubles it)
- Height range checked for live-height policies
- Job script hygiene: no hardcoded paths, no scheduler commands, env vars declared before work starts

## Testing Discipline

**Hard Split:** Unit vs Integration
- Guard test in `tests/unit/test_utils.py` (if exists) or enforced by imports
- Unit tests CANNOT import `mujoco.mjx` or instantiate environments
- Integration tests MUST pay JAX compilation cost
- This split ensures edit-loop speed: `./training/run.sh test` runs in seconds

**Test Determinism:**
```python
def test_determinism(policy):
    seq_a = []
    policy.reset()
    for _ in range(10):
        seq_a.append(
            policy.step(np.zeros(3), [0, 0, -1.0], policy.home_ctrl,
                        np.zeros(12), [0.2, 0, 0])
        )
    policy.reset()
    for i in range(10):
        t = policy.step(...)
        assert np.allclose(t, seq_a[i])
```

---

*Testing analysis: 2026-09-05*
