# `src/sim/sensors.jl` — Sensor model

## Purpose

Extracts observations from MuJoCo state, with optional noise for realism.

## Exports

| Symbol | Description |
|--------|-------------|
| `read_position` | Extract [x, y] from qpos |
| `read_observation` | Position with optional Gaussian noise |

## `read_observation(qpos; obs_noise, rng)`

- Reads position from `qpos[1:2]`
- If `obs_noise > 0`, adds `obs_noise * randn(2)` for noisy observations
- Returns [x, y] as the observation
