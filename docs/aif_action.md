# `src/aif/action.jl` — Action representation

## Purpose

Converts abstract 3D velocity actions into MuJoCo control signals.

## Exports

| Symbol | Description |
|--------|-------------|
| `to_control` | Map action to [vx, vy, vz] with scaling and clamping |
| `clamp_action` | Clamp action to limits |

## `to_control(action; scale, ctrl_lim)`

- Scales the action by `scale`
- Clamps to `[-ctrl_lim, ctrl_lim]` (default `ctrl_lim=1.2` to reduce overshooting)
- Returns control vector for MuJoCo position actuators
