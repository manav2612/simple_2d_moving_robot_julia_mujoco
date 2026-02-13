# `src/aif/beliefs.jl` — Belief state

## Purpose

Represents the agent's belief over 3D position as a Gaussian with diagonal covariance.

## Exports

| Symbol | Description |
|--------|-------------|
| `BeliefState` | Struct with mean and cov |
| `init_belief` | Create initial belief |
| `update_belief!` | Bayesian update with observation |
| `entropy` | Entropy of belief |

## BeliefState

- `mean`: [x, y, z]
- `cov`: [σ²_x, σ²_y, σ²_z] diagonal covariance

## Update

Uses precision-based Bayesian update when observing position with Gaussian noise.
