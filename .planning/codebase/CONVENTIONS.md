# Coding Conventions

**Analysis Date:** 2026-09-05

## Naming Patterns

**Files:**
- Python modules: `snake_case` (e.g., `battery.py`, `policy.py`, `env.py`)
- Test files: `test_*.py` or `*_test.py` (e.g., `test_battery.py`, `test_policy.py`)
- Package directories: `snake_case` (e.g., `wojtek_rl`, `wojtek_eval`, `wojtek_policy`)
- Configuration files: explicit purpose names (e.g., `joint_map.yaml`, `policy_meta.json`)

**Functions:**
- Regular functions: `snake_case` with descriptive names (e.g., `vibration_index`, `diag_corr`, `lateral_corr`, `height_anchor`)
- Private/internal functions: leading underscore `_snake_case` (e.g., `_contact_corr`, `_fetch_into_store`, `_is_commit`)
- Boolean-returning functions: `is_`, `has_`, `uses_`, `enabled` patterns (e.g., `uses_imu`, `tau_ff_enabled`)
- Test functions: `test_` prefix with descriptive behavior (e.g., `test_obs_shapes`, `test_pd_hold_keeps_robot_up`)

**Variables:**
- Regular variables: `snake_case` (e.g., `qpos`, `qvel`, `ctrl_dt`, `episode_length`)
- Loop counters: single lowercase letter accepted for brevity (e.g., `i`, `j` in loops)
- Numpy arrays: descriptive `snake_case` (e.g., `home_ctrl`, `anchor_ctrl`, `motor_targets`)
- JAX arrays: `snake_case` with type hints where relevant

**Classes:**
- Classes: `PascalCase` (e.g., `WojtekPolicy`, `WojtekEnv`, `WojtekJoystick`, `JointMap`)
- Exceptions: `PascalCase` with `Error` suffix (e.g., following standard Python conventions)

**Constants:**
- Module-level constants: `UPPER_SNAKE_CASE` (e.g., `OBS_SIZE`, `PRIVILEGED_SIZE`, `SCHEMA_VERSION`, `ABDUCTION_ACTUATORS`, `KNOWN_COMPONENTS`)
- Physical/model constants: `UPPER_SNAKE_CASE` (e.g., `TROT_PHASE`, `WALK_PHASE`, `HEIGHT_TABLE`, `DSECOND_TABLE`)

## Code Style

**Formatting:**
- Line length: no explicit limit enforced; typically 80-100 characters
- Indentation: 4 spaces (Python standard)
- No explicit formatter configured; code follows PEP 8 conventions

**Linting:**
- Tool: No explicit linter found; development deps include `ruff>=0.14.1` but no config present
- Code follows implicit standards observed in existing modules

**Import Organization:**

Order observed:
1. `"""Module docstring"""` at top
2. Standard library imports (e.g., `import argparse`, `from pathlib import Path`)
3. Third-party framework imports (e.g., `import jax`, `import numpy as np`)
4. JAX-specific imports (e.g., `import jax.numpy as jp` — note `jp` alias convention)
5. Application imports (e.g., `from wojtek_rl import env`, `from wojtek_rl.base import WojtekEnv`)

**Path Aliases:**
- Relative imports within package: `from wojtek_rl import module` or `from . import module`
- No global path aliases observed; imports are explicit module paths

## Error Handling

**Patterns:**
- Explicit exception types raised with descriptive messages (e.g., `ValueError`, `FileNotFoundError`, `RuntimeError`)
- Validation at entry points (e.g., policy schema version check, contract enforcement)
- Guard tests that fail early: `assert declared, f"{path.name} declares no inputs"`
- Exceptions include context: file path, schema version, contract details
- Contract validation in class constructors prevents invalid runtime states
- No silent failures; all errors raise with explanatory messages

**Example:**
```python
if version != SCHEMA_VERSION:
    raise ValueError(
        f"policy_meta.json at {meta_path} has schema_version "
        f"{version!r}, this runtime needs {SCHEMA_VERSION}"
    )
```

## Logging

**Framework:** `loguru` (imported as needed; not used everywhere; `print`/`logging` alternative)

**Patterns:**
- Sparse logging; main computation avoids excessive log output
- High-level summary logging (e.g., battery scenarios, evaluation results)
- No DEBUG/INFO/WARNING split consistently applied
- Error cases logged before exception

## Comments

**When to Comment:**
- Complex mathematical operations (e.g., contact correlation formulas, spectral analysis)
- Non-obvious model assumptions (e.g., "gravity[:, 1] = -sin(phi) for a +phi deg roll")
- Context for constants (e.g., "Actuator order is per leg (abduction, hip, knee)")
- Experimental or temporary workarounds clearly marked
- Guard conditions explained when not self-evident

**JSDoc/TSDoc:**
- Not used for Python; docstrings follow Google/NumPy style
- Function docstrings include: one-line summary, full description, parameter details (where non-obvious), return type/description
- Module docstrings document purpose, usage patterns, and key references

**Example:**
```python
def target_sag_cost(
    qpos_act: jax.Array, motor_targets: jax.Array, contact: jax.Array
) -> jax.Array:
    """Squared gap between commanded motor targets and measured joint
    positions, summed over the legs whose foot is in ground contact.

    On a soft plant (low kp) gravity makes the loaded joints sag below
    their targets by tau/kp; offsetting the targets upward cannot close
    this gap (the offset IS a target != position).
    """
```

## Function Design

**Size:** Functions typically 10-40 lines; some utilities (e.g., `_contact_corr`, `band_power_fraction`) are 5-10 lines

**Parameters:** 
- Positional arguments preferred for required inputs (e.g., `def step(state, action)`)
- Keyword arguments for optional configuration (e.g., `cutoff_hz=5.0`, `dt=0.02`)
- Type hints used consistently (e.g., `qpos_act: jax.Array`, `contact: jax.Array`)
- Avoid mutable defaults

**Return Values:**
- Single return value common (e.g., `float`, `jax.Array`, result object)
- Multiple returns as tuple when necessary (e.g., `return env, reset_state`)
- Structured objects (e.g., `policy.step()` returns array, stores intermediate state in `policy.last_obs`)

## Module Design

**Exports:**
- Clear public API via top-level functions and classes
- Private helpers prefixed with `_` (e.g., `_contact_corr`, `_fetch_into_store`)
- No `__all__` export list observed; relies on naming convention

**Barrel Files:**
- Not used; imports are explicit from modules (e.g., `from wojtek_rl.battery import vibration_index`)
- `__init__.py` files minimal (often empty or with version info)

**Module Cohesion:**
- One responsibility per module (e.g., `battery.py` for evaluation metrics, `policy.py` for runtime, `env.py` for training environment)
- Shared constants extracted to base modules (e.g., `base.py`, `paths.py`)
- Clear import dependencies; avoid circular imports

## Naming Conventions Applied to Project

**Repository naming:** "Wojtek" everywhere (robotics context). The project uses:
- `wojtek_rl`: training/RL module
- `wojtek_eval`: evaluation module
- `wojtek_policy`: policy runtime (ROS)
- `wojtek_*` packages: domain-specific ROS packages
- Legacy `fbb_*` names removed from active code (Apache 2.0 license applied as default)

---

*Convention analysis: 2026-09-05*
