# `src/aif/efe.jl` — Expected Free Energy

## Purpose

Computes Expected Free Energy for policy selection. Lower EFE = better action. All terms use explicit 3D (x, y, z) per-dimension formulation.

## Exports

| Symbol | Description |
|--------|-------------|
| `compute_efe` | Full EFE for an action |
| `pragmatic_term` | Goal-distance cost |
| `epistemic_term` | Uncertainty/information term |

## Formula

EFE = pragmatic (goal-seeking) − epistemic (exploration)

- **Pragmatic**: `γ * Σ w_i (pred_i - goal_i)²` where `pred = μ + clamp(action × ctrl_scale, -ctrl_lim, ctrl_lim)`. Uses actual applied control for correct prediction.
- **Epistemic**: `−β * Σ log(σ²_i)` with clamped covariance for numerical stability.
- **axis_weights**: Optional per-axis weights `[w_x, w_y, w_z]` for goal distance.
