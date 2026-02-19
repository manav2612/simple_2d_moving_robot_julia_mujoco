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
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500 --ctrl_scale 3.0 --save_plot trajectory.png
```

## CLI Usage

```bash
julia --project=. scripts/run_cli.jl --goal <x> <y> <z> --init <x> <y> <z> [options]
```

| Option | Description | Default |
|--------|-------------|---------|
| `--goal` | Goal position (x y z) | 0.8 0.8 0.4 |
| `--init` | Initial position (x y z) | -0.5 -0.5 0.2 |
| `--robot` | Robot/arm starting position (x y z), used by `--panda` | 0.6 0.0 0.4 |
| `--steps` | Max simulation steps | 500 |
| `--ctrl_scale` | Control scaling | 3.0 |
| `--obs_noise` | Observation variance (σ²) | 0.005 |
| `--process_noise` | Process noise variance | 0.002 |
| `--alpha` | EMA smoothing weight (0–1; lower = smoother) | 0.3 |
| `--save_plot` | Path to save trajectory plot | (none) |
| `--render` | Launch MuJoCo visualiser | false |
| `--panda` | AIF-driven Panda arm (implies `--render`) | false |
| `--renderarm` | Panda arm replay + scene | false |
| `--speed` | Control steps/sec for visualiser (0 = no limit) | 0 |
| `--save_gif` | Save simulation as animated GIF (path) | "" |
| `--save_mp4` | Save simulation as MP4 video (path) | "" |
| `--backend` | Inference backend: `analytic` or `rxinfer` | analytic |
| `--agent_color` | Agent geom color r g b [a] | (default) |
| `--goal_color` | Goal marker color r g b [a] | (default) |
| `--seed` | Random seed | 42 |
| `--verbose` | Print step logs | true |

GIF tuning: `--gif_size W H`, `--gif_fps`, `--gif_stride`, `--gif_wp_frames`, `--gif_extra_frames`.
MP4 tuning: `--mp4_size W H`, `--mp4_fps`, `--mp4_stride`, `--mp4_wp_frames`, `--mp4_extra_frames`.

## Panda AIF Mode (`--panda`)

AIF-driven Panda arm where the controller drives the arm through a pick-and-place sequence:

1. **Reach**: Arm moves from `--robot` to `--init` (the object)
2. **Pick**: Arm dwells at init; red ball attaches to arm
3. **Drop**: AIF drives arm from init to `--goal`; red ball follows
4. **Place**: Red ball placed on top of green box at goal

```bash
julia --project=. scripts/run_cli.jl --panda --speed 4 --robot 0.6 0 0.4 --init 0.4 0 0.2 --goal 0.6 0.2 0.35 --steps 100
```

## Render Arm (`--renderarm`)

Replays a pre-computed AIF trajectory on a Panda arm with a pick-and-place scene:

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
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 3.0 --save_plot trajectory.png --renderarm
```

## MuJoCo GUI Viewer

Install the visualiser (run once):

```bash
julia --project=. -e 'using MuJoCo; MuJoCo.install_visualiser()'
```

Run with render:

```bash
julia --project=. scripts/run_cli.jl --render --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500 --ctrl_scale 3.0 --save_plot trajectory.png
```

Requires a display (WSLg or X11 on WSL2). When no display is available, the CLI automatically falls back to headless simulation.

## Save GIF (`--save_gif`)

Saves the simulation as an animated GIF using offscreen rendering. Works with `--panda`, `--render`, and `--renderarm`. The GIF is written after the simulation completes (close the visualiser or wait for steps to finish).

**Requirements**: Display (DISPLAY) for OpenGL; ffmpeg (installed via visualiser deps).

```bash
# Panda AIF simulation → GIF
julia --project=. scripts/run_cli.jl --panda --speed 4 --robot 0.6 0 0.4 --init 0.4 0 0.2 --goal 0.6 0.2 0.35 --steps 100 --save_gif simulation.gif

# With renderarm (trajectory replayed on Panda arm)
julia --project=. scripts/run_cli.jl --renderarm --goal 0.8 0.8 0.4 --init 0.3 0 0.2 --steps 500 --save_gif trajectory.gif
```

## Save MP4 (`--save_mp4`)

Saves the simulation as an H.264 MP4 video. Faster encoding than GIF with better quality.

```bash
julia --project=. scripts/run_cli.jl --renderarm --goal 0.8 0.8 0.4 --init 0.3 0 0.2 --steps 500 --save_mp4 trajectory.mp4
```

## Project Structure

| Path | Description |
|------|-------------|
| `src/AIFMuJoCoRobot.jl` | Main module |
| `src/aif/` | Active Inference (beliefs, EFE, policy, action, RxInfer filter) |
| `src/sim/` | MuJoCo env, Panda env, and sensors |
| `src/control/` | AIF controller |
| `models/robot.xml` | MuJoCo 3D robot model |
| `panda_render_scene.xml` | Panda pick-and-place scene |
| `experiments/` | Configs and run scripts |
| `docs/` | Per-file documentation |

## Per-File Documentation

- `docs/run_cli.md` - CLI usage, Panda AIF mode, render arm, GIF/MP4 export
- `docs/aif_beliefs.md` - Belief state (per-dimension noise, covariance bounds)
- `docs/aif_efe.md` - Expected Free Energy (ctrl_scale in prediction)
- `docs/aif_policy.md` - Policy selection (7×7×7 action set, EMA smoothing)
- `docs/aif_action.md` - Action representation
- `docs/aif_generative_model.md` - Generative model
- `docs/aif_rxinfer_filter.md` - RxInfer streaming filter backend
- `docs/control_aif_controller.md` - AIF controller
- `docs/sim_sensors.md` - Sensors
- `docs/sim_mujoco_env.md` - MuJoCo environment
- `docs/sim_panda_env.md` - Panda arm environment
- `docs/utils_math.md` - Math utilities
- `docs/utils_logging.md` - Logging
- `docs/models.md` - MuJoCo models

## RxInfer Backend (`--backend rxinfer`)

The project supports an optional **RxInfer.jl** streaming inference backend that replaces the built-in analytic diagonal-Gaussian belief updates with reactive message-passing on a factor graph.

### How it works

Each axis (x, y, z) runs an independent 1D linear-Gaussian state-space model via RxInfer's online streaming engine:

- **Transition**: `x ~ Normal(mean = x_prev + u, variance = q)` where `u` is the applied control
- **Observation**: `y ~ Normal(mean = x, variance = r)`
- **Autoupdates**: the posterior from each step becomes the prior for the next step

The RxInfer posterior is mathematically equivalent to the analytic Kalman-filter-style update (verified by tests to machine-epsilon accuracy).

### Usage

```bash
# Run with RxInfer backend (headless)
julia --project=. scripts/run_cli.jl --backend rxinfer --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500

# Default analytic backend (unchanged behaviour)
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 --steps 500
```

### Programmatic use

```julia
using AIFMuJoCoRobot

# Use RxInfer backend in run_simulation
result = run_simulation(;
    goal = [0.8, 0.8, 0.4],
    init_pos = [-0.5, -0.5, 0.2],
    inference_backend = :rxinfer,   # :analytic (default) or :rxinfer
    action_alpha = 0.3,             # EMA smoothing (0-1; lower = smoother)
)
```

### Standalone filter (without MuJoCo)

```julia
using AIFMuJoCoRobot

filter = AIFMuJoCoRobot.RxInferFilter.init_rxinfer_filter(
    [0.0, 0.0, 0.0], [0.01, 0.01, 0.01];
    obs_noise = 0.01, process_noise = 0.005,
)
post_mean, post_var = AIFMuJoCoRobot.RxInferFilter.rxinfer_step!(filter, ctrl, obs)
AIFMuJoCoRobot.RxInferFilter.rxinfer_stop!(filter)
```

### Future extensions (Phase B/C)

- **Online noise learning**: extend the RxInfer model to place Gamma priors on observation/process precision and learn them online with variational constraints.
- **Bethe Free Energy monitoring**: expose RxInfer's free energy stream for comparing with the hand-coded EFE values used for policy selection.

## Smooth Trajectories

The default configuration is tuned for smooth, jitter-free trajectories. Key mechanisms:

1. **EMA control smoothing** (`action_alpha=0.3`): Exponential Moving Average blends 30% new control with 70% previous, eliminating discrete action jumps.
2. **Fine action grid** (7×7×7 = 343 actions, `step_size=0.04`): Reduces quantization artifacts compared to the previous 5×5×5 grid.
3. **Joint damping** (`damping=15`) + moderate actuator gain (`kp=80`): Critically-damped physics prevents oscillation.
4. **Low noise** (`obs_noise=0.005`, `process_noise=0.002`): Stable beliefs lead to consistent action selection.

Adjust `--alpha` to control smoothness: `0.15` = ultra-smooth, `0.3` = default, `1.0` = no smoothing.

## AIF Notes

- Belief prediction uses **actual applied control** (ctrl), not raw action, for correct dynamics.
- Per-dimension `obs_noise` and `process_noise` supported (scalar or `[σ²_x, σ²_y, σ²_z]`).
- Covariance bounds prevent numerical instability.
- Default config: γ=1.5, β=0.02, ctrl_scale=3.0, action_alpha=0.3 for smooth goal-reaching.

## References

- MuJoCo.jl: https://github.com/JamieMair/MuJoCo.jl
- RxInfer.jl: https://github.com/ReactiveBayes/RxInfer.jl
