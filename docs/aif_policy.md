# `src/aif/policy.jl` — Policy selection

## Purpose

Selects the action that minimizes Expected Free Energy. Supports per-axis weights and correct control scaling for prediction.

## Exports

| Symbol | Description |
|--------|-------------|
| `select_action` | Choose best action given belief and goal |
| `get_action_set` | Discrete 125-action set (5×5×5 grid in 3D) |

## Action set

125 velocity actions: all combinations of (dx, dy, dz) with each in `{-s, -s/2, 0, s/2, s}` and default `step_size=0.08` for finer control and smoother goal-reaching.
