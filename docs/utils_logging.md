# `src/utils/logging.jl` — Logging utilities

## Purpose

Simple logging for simulation steps and summaries using Printf.

## Exports

| Symbol | Description |
|--------|-------------|
| `log_step` | Log one simulation step |
| `log_summary` | Log simulation summary |

## Functions

- `log_step(step, pos, goal, belief_mean, efe)` — prints step index, position, goal, belief, EFE
- `log_summary(total_steps, final_dist, converged)` — prints total steps, final distance, convergence
