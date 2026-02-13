# AIF MuJoCo Robot

A **simple** Active Inference (AIF) controller for a 3D MuJoCo robot. The robot navigates toward a goal by minimizing Expected Free Energy (EFE).

## Overview

- **MuJoCo**: Physics simulation with a 3D point-mass robot (slide joints in x, y, z)
- **Active Inference**: Belief state, generative model, EFE-based policy selection, per-dimension (x,y,z) noise
- **Julia**: Implemented in Julia using MuJoCo.jl

## Quick Start

```bash
cd aif_mujoco_robot
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500 --ctrl_scale 5.0 --save_plot trajectory.png
```

## CLI Usage

```bash
julia --project=. scripts/run_cli.jl --goal <x> <y> <z> --init <x> <y> <z> [options]
```

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
| `--renderarm` | Panda arm replay + scene | false |

## Render Arm (`--renderarm`)

Replays the AIF trajectory on a Panda arm with a pick-and-place scene:

- **Red sphere** at initial position (from `--init`)
- **Green box** at goal position (from `--goal`)

**Sequence**: (1) Arm moves to init → pickup red ball. (2) Red ball follows arm along AIF trajectory → carry. (3) Red ball placed on top of green box → drop. Trajectory is prepended with init so the arm first goes to the red ball.

**Output bounds** (Panda arm workspace; trajectory is clamped):

| Axis | Min | Max |
|------|-----|-----|
| X | 0.2 | 0.8 |
| Y | -0.4 | 0.4 |
| Z | 0.1 | 0.5 |

**Recommended input bounds** for `--goal` and `--init`: same as above.

```bash
# Use init within Panda bounds (0.2–0.8, -0.4–0.4, 0.1–0.5) for pickup to work
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 5.0 --save_plot trajectory.png --renderarm
```

## MuJoCo GUI Viewer

Install the visualiser (run once):

```bash
julia --project=. -e 'using MuJoCo; MuJoCo.install_visualiser()'
```

Run with render:

```bash
julia --project=. scripts/run_cli.jl --render --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500 --save_plot trajectory.png
```

Requires a display (WSLg or X11 on WSL2).

## Project Structure

| Path | Description |
|------|-------------|
| `src/AIFMuJoCoRobot.jl` | Main module |
| `src/aif/` | Active Inference (beliefs, EFE, policy, action) |
| `src/sim/` | MuJoCo env and sensors |
| `src/control/` | AIF controller |
| `models/robot.xml` | MuJoCo 3D robot model |
| `experiments/` | Configs and run scripts |
| `docs/` | Per-file documentation |

## Per-File Documentation

- `docs/run_cli.md` - CLI usage and render arm (pickup, carry, drop)
- `docs/aif_beliefs.md` - Belief state (per-dimension noise, covariance bounds)
- `docs/aif_efe.md` - Expected Free Energy (ctrl_scale in prediction)
- `docs/aif_policy.md` - Policy selection (5×5×5 action set)
- `docs/aif_action.md` - Action representation
- `docs/aif_generative_model.md` - Generative model
- `docs/control_aif_controller.md` - AIF controller
- `docs/sim_sensors.md` - Sensors
- `docs/sim_mujoco_env.md` - MuJoCo environment
- `docs/utils_math.md` - Math utilities
- `docs/utils_logging.md` - Logging
- `docs/models.md` - MuJoCo models

## AIF Notes

- Belief prediction uses **actual applied control** (ctrl), not raw action, for correct dynamics.
- Per-dimension `obs_noise` and `process_noise` supported (scalar or `[σ²_x, σ²_y, σ²_z]`).
- Covariance bounds prevent numerical instability.
- Default config: γ=1.2, β=0.05, process_noise=0.005 for stable goal-reaching.

## References

- MuJoCo.jl: https://github.com/JamieMair/MuJoCo.jl
