# `src/aif/generative_model.jl` — Generative model

## Purpose

Defines transition and observation models for Active Inference. 3D state (x, y, z) with per-dimension noise.

## Exports

| Symbol | Description |
|--------|-------------|
| `predict_transition` | s' = s + u |
| `observation_likelihood` | P(o \| s) diagonal Gaussian |
| `predict_observation` | Expected observation from state |

## Transition

s_{t+1} = s_t + u_t (additive dynamics; state and action are 3D [x, y, z])

## Observation

`observation_likelihood(obs, state; obs_noise)` — Per-dimension Gaussian. `obs_noise` can be scalar or `[σ²_x, σ²_y, σ²_z]`. Returns product of per-axis likelihoods.
