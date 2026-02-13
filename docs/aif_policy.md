# `src/aif/policy.jl` — Policy selection

## Purpose

Selects the action that minimizes Expected Free Energy.

## Exports

| Symbol | Description |
|--------|-------------|
| `select_action` | Choose best action given belief and goal |
| `get_action_set` | Discrete 27-action set (3×3×3 grid in 3D) |

## Action set

27 velocity actions: all combinations of (dx, dy, dz) with each in {-s, 0, s} and default step_size=0.15.
