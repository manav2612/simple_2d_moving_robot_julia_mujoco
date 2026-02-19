# `src/sim/sensors.jl` — Sensor model

## Purpose

Extracts observations from MuJoCo state, with optional per-dimension noise.

## Exports

| Symbol | Description |
|--------|-------------|
| `read_position` | Extract [x, y, z] from qpos |
| `read_observation` | Position with optional Gaussian noise |

## `read_position(qpos)`

Returns `[qpos[1], qpos[2], qpos[3]]`.

## `read_observation(qpos; obs_noise, rng)`

- Reads position from `qpos[1:3]`
- `obs_noise`: scalar or `[σ²_x, σ²_y, σ²_z]` — variance per axis (default `0.0`)
- `rng`: random number generator (default `nothing` — returns noiseless position when `nothing`)
- When `rng` is provided and `σ²_i > 0`, adds `sqrt(σ²_i) * randn(rng)` per dimension
- Returns [x, y, z] as the observation
