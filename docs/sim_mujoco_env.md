# `src/sim/mujoco_env.jl` — MuJoCo environment

## Purpose

Wraps MuJoCo for the 3D robot: load model, reset, step, and read state.

## Exports

| Symbol | Description |
|--------|-------------|
| `EnvState` | Struct with model, data, goal |
| `load_env` | Load model from XML path |
| `reset!` | Reset to initial position |
| `step!` | Apply control and step simulation |
| `get_position` | Current [x, y, z] from qpos |
| `get_goal` | Goal [x, y, z] |

## Model layout

- `qpos[1]`, `qpos[2]`, `qpos[3]`: x, y, z (slide joints)
- `ctrl[1]`, `ctrl[2]`, `ctrl[3]`: displacement (target = pos + ctrl) for x, y, z

## `step!(env, ctrl; nsteps)`

Applies `ctrl` and runs `nsteps` MuJoCo steps (default 5) for stability.
