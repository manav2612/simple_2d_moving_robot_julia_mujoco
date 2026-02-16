# `src/sim/sensors.jl` — Sensor model

## Purpose

Extracts observations from MuJoCo state, with optional per-dimension noise.

## Exports

| Symbol | Description |
|--------|-------------|
| `read_position` | Extract [x, y, z] from qpos |
| `read_observation` | Position with optional Gaussian noise |

## `read_observation(qpos; obs_noise, rng)`

- Reads position from `qpos[1:3]`
- `obs_noise`: scalar or `[σ²_x, σ²_y, σ²_z]` — variance per axis
- Adds `sqrt(σ²_i) * randn()` per dimension when σ²_i > 0
- Returns [x, y, z] as the observation
