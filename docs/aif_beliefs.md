# `src/aif/beliefs.jl` — Belief state

## Purpose

Represents the agent's belief over 3D position (x, y, z) as a Gaussian with diagonal covariance. Supports per-dimension observation and process noise.

## Exports

| Symbol | Description |
|--------|-------------|
| `BeliefState` | Struct with mean and cov |
| `init_belief` | Create initial belief |
| `update_belief!` | Bayesian update with observation |
| `predict_belief!` | Predict belief forward given control |
| `entropy` | Entropy of belief |

## BeliefState

- `mean`: [x, y, z]
- `cov`: [σ²_x, σ²_y, σ²_z] diagonal covariance (bounded to `[1e-5, 2.0]` for stability)

## Update

`update_belief!(b, obs; obs_noise)` — Per-dimension Bayesian update. `obs_noise` can be scalar or `[σ²_x, σ²_y, σ²_z]`. Posterior precision = prior_precision + likelihood_precision per axis.

## Predict

`predict_belief!(b, ctrl; process_noise)` — Predicts belief using **actual applied control** (not raw action). Uses transition `s' = s + ctrl`. `process_noise` can be scalar or `[σ²_x, σ²_y, σ²_z]`. Covariance is bounded to prevent runaway uncertainty.

## Alternative: RxInfer backend

When `inference_backend = :rxinfer` is set, the simulation loop bypasses `predict_belief!` and `update_belief!` and instead routes both steps through the RxInfer streaming filter (`src/aif/rxinfer_filter.jl`). The RxInfer posterior is written back into `belief.mean` and `belief.cov` so the rest of the pipeline (EFE, policy, controller) remains unchanged. See `docs/aif_rxinfer_filter.md` for details.
