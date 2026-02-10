# `src/aif/policy.jl` — Policy selection

## Purpose

Selects the action that minimizes Expected Free Energy.

## Exports

| Symbol | Description |
|--------|-------------|
| `select_action` | Choose best action given belief and goal |
| `get_action_set` | Discrete 9-action set (3x3 grid in 2D) |

## Action set

9 velocity actions: (-s,-s), (0,-s), (s,-s), (-s,0), (0,0), (s,0), (-s,s), (0,s), (s,s) with default step_size=0.15.
