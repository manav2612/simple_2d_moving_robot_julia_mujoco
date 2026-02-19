# `src/sim/panda_env.jl` — Panda arm environment

## Purpose

Panda arm environment adapter for AIF-driven control. Maps AIF's 3D control signals to mocap updates for the Panda end-effector. Manages the red ball (pick-up object) and arm mocap positions.

## Exports

| Symbol | Description |
|--------|-------------|
| `PandaEnvState` | Struct holding model, data, positions, mocap IDs |
| `load_panda_env` | Load Panda scene from XML, substitute positions |

Methods `reset!`, `step!`, `get_position`, and `get_goal` are defined but not exported (to avoid conflict with `MuJoCoEnv`). They are dispatched on `PandaEnvState` via the parent module.

## PandaEnvState

```julia
struct PandaEnvState
    model::MuJoCo.Model
    data::MuJoCo.Data
    robot_pos::Vector{Float64}     # arm starting position
    init_pos::Vector{Float64}      # red object position (pick-up)
    goal::Vector{Float64}          # green box position (place)
    mocap_row::Int                 # panda_mocap body row in mocap_pos
    red_mocap_row::Int             # red_object body row in mocap_pos
    ee_site_id::Int                # ee_center_site ID for reading EE position
    mocap_gain::Float64            # interpolation gain for mocap updates
end
```

## `load_panda_env(path; robot_pos, init_pos, goal, mocap_gain)`

Loads the Panda render scene XML, substituting `__INIT_POS__` and `__GOAL_POS__` placeholders with the provided positions. Initializes the arm actuators to match current joint positions and resolves mocap body IDs for `panda_mocap` and `red_object`.

| Parameter | Default |
|-----------|---------|
| `robot_pos` | `[0.6, 0.0, 0.4]` |
| `init_pos` | `[0.4, 0.0, 0.2]` |
| `goal` | `[0.6, 0.2, 0.35]` |
| `mocap_gain` | `0.02` |

## `reset!(env; robot_pos, init_pos, goal)`

Resets MuJoCo state. Sets the panda mocap to `robot_pos` and the red object mocap to `init_pos`. All parameters are optional (pass `nothing` to keep current values).

## `step!(env, ctrl; nsteps)`

Computes a target position as `ee_pos + ctrl[1:3]`, then smoothly interpolates the panda mocap toward the target using `mocap_gain`. Runs `nsteps` MuJoCo physics steps (default 5).

## `get_position(env)`

Returns the end-effector position from `site_xpos` (uses `ee_center_site`), not from mocap directly.

## `get_goal(env)`

Returns a copy of the goal vector.
