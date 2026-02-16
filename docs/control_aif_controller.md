# `src/control/aif_controller.jl` — AIF controller

## Purpose

Connects belief, policy, and action: computes control from belief and goal, and returns EFE for logging. Belief prediction uses **actual applied control** (ctrl), not raw action.

## Exports

| Symbol | Description |
|--------|-------------|
| `compute_control` | Main entry: belief + goal → control + EFE |

## `compute_control(belief, goal; γ, β, ctrl_scale, axis_weights, ctrl_lim)`

Belief and goal are 3D position vectors [x, y, z].

1. Call `Policy.select_action` (with `ctrl_scale` for correct EFE prediction)
2. Convert action to control with `Action.to_control`
3. Compute EFE for the chosen action
4. Return `(ctrl, action, efe)`

The simulation loop calls `predict_belief!(belief, ctrl)` — using the scaled control, not the raw action — so the belief matches the actual dynamics.

## EMA smoothing

The raw control from `compute_control` is smoothed in `run_simulation` via Exponential Moving Average before being applied:

```
ctrl_smoothed = α × ctrl_new + (1 − α) × ctrl_previous
```

The `action_alpha` parameter (default 0.3) controls the blend. This smoothing happens in the simulation loop (not in the controller itself), so `compute_control` remains a pure, stateless function.

## Inference backend

The controller itself is backend-agnostic: it always consumes `belief.mean` and `belief.cov`. The `inference_backend` kwarg in `run_simulation` controls whether those values come from the analytic update (`predict_belief!` + `update_belief!`) or the RxInfer streaming filter. See `docs/aif_rxinfer_filter.md`.
