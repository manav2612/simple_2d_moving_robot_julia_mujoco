# `scripts/run_cli.jl` — CLI and Render Arm

## Purpose

Command-line interface for running AIF simulations and visualising results. Supports MuJoCo render and Panda arm pick-and-place replay.

## Usage

```bash
julia --project=. scripts/run_cli.jl --goal <x> <y> <z> --init <x> <y> <z> [options]
```

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--goal` | Goal position (x y z) | 0.8 0.8 0.4 |
| `--init` | Initial position (x y z) | -0.5 -0.5 0.2 |
| `--steps` | Max simulation steps | 500 |
| `--ctrl_scale` | Control scaling | 4.0 |
| `--obs_noise` | Observation variance (σ²) | 0.01 |
| `--process_noise` | Process noise variance | 0.005 |
| `--save_plot` | Path to save trajectory plot | (none) |
| `--render` | Launch MuJoCo visualiser | false |
| `--renderarm` | Panda arm replay + pick-and-place scene | false |
| `--backend` | Inference backend: `analytic` or `rxinfer` | analytic |
| `--agent_color` | Agent geom color r g b [a] | (default) |
| `--goal_color` | Goal marker color r g b [a] | (default) |
| `--seed` | Random seed | 42 |
| `--verbose` | Print step logs | true |

## Render Arm (`--renderarm`)

Replays the AIF trajectory on a Panda arm with a pick-and-place scene.

### Scene

- **Red sphere** at `--init` (the object to pick up)
- **Green box** at `--goal` (the place target)

### Sequence

1. **Pickup**: Arm moves to init position (first waypoint). Red ball stays at init.
2. **Carry**: When arm reaches init, red ball follows arm end effector along the AIF trajectory.
3. **Drop**: When trajectory completes, red ball is placed on top of the green box.

### Trajectory

The Panda trajectory is `[init_clamped, pos_1, pos_2, ...]` — init is prepended so the arm first goes to the red ball. All positions are clamped to the Panda workspace.

### Bounds

**Output** (Panda arm workspace):

| Axis | Min | Max |
|------|-----|-----|
| X | 0.2 | 0.8 |
| Y | -0.4 | 0.4 |
| Z | 0.1 | 0.5 |

**Recommended input** for `--goal` and `--init`: same as above.

### Example

Use init/goal within Panda bounds (X 0.2–0.8, Y -0.4–0.4, Z 0.1–0.5) so the arm can reach both the red ball and green box:

```bash
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 5.0 --save_plot trajectory.png --renderarm
```

If init is outside bounds (e.g. -0.5 -0.5 0.2), the arm goes to the clamped position; the red ball stays at init and may not align with the arm for pickup.

## Inference Backend (`--backend`)

Controls which belief-update engine is used during simulation:

- `analytic` (default): built-in closed-form diagonal-Gaussian Kalman update (`predict_belief!` + `update_belief!`)
- `rxinfer`: RxInfer.jl streaming message-passing filter (per-axis 1D linear-Gaussian state-space model)

Both produce mathematically identical posteriors. The RxInfer backend is useful for experimenting with richer probabilistic models (e.g. online noise learning) in the future.

### Example with RxInfer + Panda arm

```bash
julia --project=. scripts/run_cli.jl --backend rxinfer --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 5.0 --save_plot trajectory.png --renderarm
```
