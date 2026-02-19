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

## EnvState

```julia
struct EnvState
    model::MuJoCo.Model
    data::MuJoCo.Data
    goal::Vector{Float64}
end
```

## Model layout

- `qpos[1]`, `qpos[2]`, `qpos[3]`: x, y, z (slide joints)
- `ctrl[1]`, `ctrl[2]`, `ctrl[3]`: target position for x, y, z (computed as `pos + ctrl`)

## `load_env(model_path; goal)`

Loads a MuJoCo XML model and initializes the simulation data. Default goal is `[0.8, 0.8, 0.4]`.

## `reset!(env; init_pos)`

Resets MuJoCo state and writes `init_pos` into `qpos[1:3]`. Default init is `[-0.5, -0.5, 0.2]`.

## `step!(env, ctrl; nsteps)`

Computes target position as `pos + ctrl`, clamps each actuator to the model's `actuator_ctrlrange` (falls back to ±10 if the range cannot be read), and runs `nsteps` MuJoCo physics steps (default 5, config default 8) for stability. More sub-steps per control produce smoother physics interpolation.

## `get_position(env)`

Returns `[qpos[1], qpos[2], qpos[3]]` — the current robot position.

## `get_goal(env)`

Returns a copy of the goal vector.
