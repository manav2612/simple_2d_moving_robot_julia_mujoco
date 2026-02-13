# `src/sim/sensors.jl` — Sensor model

## Purpose

Extracts observations from MuJoCo state, with optional noise for realism.

## Exports

| Symbol | Description |
|--------|-------------|
| `read_position` | Extract [x, y, z] from qpos |
| `read_observation` | Position with optional Gaussian noise |

## `read_observation(qpos; obs_noise, rng)`

- Reads position from `qpos[1:3]`
- If `obs_noise > 0`, adds `sqrt(obs_noise) * randn(3)` for noisy observations
- Returns [x, y, z] as the observation
