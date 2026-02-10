# `src/aif/efe.jl` — Expected Free Energy

## Purpose

Computes Expected Free Energy for policy selection. Lower EFE = better action.

## Exports

| Symbol | Description |
|--------|-------------|
| `compute_efe` | Full EFE for an action |
| `pragmatic_term` | Goal-distance cost |
| `epistemic_term` | Uncertainty/information term |

## Formula

EFE = γ * ||μ + u - goal||² - β * Σ log(σ²)

- Pragmatic: penalizes distance to goal
- Epistemic: favors uncertainty reduction
