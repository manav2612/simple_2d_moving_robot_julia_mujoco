# `src/aif/policy.jl` — Policy selection

## Purpose

Selects the action that minimizes Expected Free Energy. Supports per-axis weights and correct control scaling for prediction.

## Exports

| Symbol | Description |
|--------|-------------|
| `select_action` | Choose best action given belief and goal |
| `get_action_set` | Discrete 343-action set (7×7×7 grid in 3D) |

## Action set

343 velocity actions: all combinations of `(dx, dy, dz)` with each in `{-3s, -2s, -s, 0, s, 2s, 3s}` and default `step_size=0.04`. The fine grid (7 values per axis) minimises quantization artifacts for smoother trajectories compared to the previous 5×5×5 grid.

Combined with the EMA control smoothing (`action_alpha`) in `run_simulation`, this produces clean, jitter-free curves.
