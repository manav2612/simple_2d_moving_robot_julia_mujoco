# `src/control/aif_controller.jl` — AIF controller

## Purpose

Connects belief, policy, and action: computes control from belief and goal, and returns EFE for logging.

## Exports

| Symbol | Description |
|--------|-------------|
| `compute_control` | Main entry: belief + goal → control + EFE |

## `compute_control(belief, goal; γ, β, ctrl_scale)`

1. Call `Policy.select_action` to get the best action
2. Convert action to control with `Action.to_control`
3. Compute EFE for the chosen action
4. Return `(ctrl, action, efe)`
