# `src/aif/generative_model.jl` — Generative model

## Purpose

Defines transition and observation models for Active Inference.

## Exports

| Symbol | Description |
|--------|-------------|
| `predict_transition` | s' = s + u |
| `observation_likelihood` | P(o | s) |
| `predict_observation` | Expected observation from state |

## Transition

s_{t+1} = s_t + u_t (additive dynamics)

## Observation

o_t = s_t + noise (Gaussian likelihood)
