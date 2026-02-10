# `src/sim/mujoco_env.jl` — MuJoCo environment

## Purpose

Wraps MuJoCo for the 2D robot: load model, reset, step, and read state.

## Exports

| Symbol | Description |
|--------|-------------|
| `EnvState` | Struct with model, data, goal |
| `load_env` | Load model from XML path |
| `reset!` | Reset to initial position |
| `step!` | Apply control and step simulation |
| `get_position` | Current [x, y] from qpos |
| `get_goal` | Goal [x, y] |

## Model layout

- `qpos[1]`, `qpos[2]`: x, y (slide joints)
- `ctrl[1]`, `ctrl[2]`: velocity commands for x, y

## `step!(env, ctrl; nsteps)`

Applies `ctrl` and runs `nsteps` MuJoCo steps (default 5) for stability.
