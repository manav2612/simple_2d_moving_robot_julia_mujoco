# `scripts/run_cli.jl` — CLI and Render Arm

## Purpose

Command-line interface for running AIF simulations and visualising results. Supports headless runs, MuJoCo GUI render, Panda arm AIF-driven control, Panda arm pick-and-place replay, and GIF/MP4 export.

## Usage

```bash
julia --project=. scripts/run_cli.jl --goal <x> <y> <z> --init <x> <y> <z> [options]
```

## Options

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
| `--renderarm` | Panda arm replay + pick-and-place scene | false |
| `--save_gif` | Save simulation as animated GIF (path) | (none) |
| `--save_mp4` | Save simulation as MP4 video (path) | (none) |
| `--backend` | Inference backend: `analytic` or `rxinfer` | analytic |
| `--speed` | Control steps per second for visualiser (0 = no limit) | 0.0 |
| `--agent_color` | Agent geom color r g b [a] | (default) |
| `--goal_color` | Goal marker color r g b [a] | (default) |
| `--seed` | Random seed | 42 |
| `--verbose` | Print step logs | true |

### GIF Tuning Options

| Option | Description | Default |
|--------|-------------|---------|
| `--gif_size` | GIF width and height (two ints) | 480 360 |
| `--gif_fps` | GIF frames per second | 10 |
| `--gif_stride` | Capture every Nth render step (2 = 2× fewer frames) | 2 |
| `--gif_wp_frames` | Max render steps per waypoint | 10 |
| `--gif_extra_frames` | Extra render steps after final waypoint | 60 |

### MP4 Tuning Options

| Option | Description | Default |
|--------|-------------|---------|
| `--mp4_size` | MP4 width and height (two ints) | 640 480 |
| `--mp4_fps` | MP4 frames per second | 30 |
| `--mp4_stride` | Capture every Nth render step | 1 |
| `--mp4_wp_frames` | Max render steps per waypoint | 10 |
| `--mp4_extra_frames` | Extra render steps after final waypoint | 60 |

## Panda AIF Mode (`--panda`)

AIF-driven Panda arm mode where the Active Inference controller drives the Panda arm directly through a pick-and-place sequence. Implies `--render`.

### Stages

1. **Reach**: Arm moves from `--robot` position toward `--init` (the object).
2. **Pick**: Arm dwells at init for a short period; red ball attaches to arm.
3. **Drop**: AIF drives the arm from init toward `--goal`; red ball follows arm.
4. **Place**: Red ball is smoothly placed on top of the green box at goal.
5. **Done**: Sequence complete.

### Example

```bash
julia --project=. scripts/run_cli.jl --panda --speed 4 --robot 0.6 0 0.4 --init 0.4 0 0.2 --goal 0.6 0.2 0.35 --steps 100
```

## Render Arm (`--renderarm`)

Replays a pre-computed AIF trajectory on a Panda arm with a pick-and-place scene (no live AIF control — the headless simulation runs first, then the trajectory is played back).

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
julia --project=. scripts/run_cli.jl --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 3.0 --save_plot trajectory.png --renderarm
```

If init is outside bounds (e.g. -0.5 -0.5 0.2), the arm goes to the clamped position; the red ball stays at init and may not align with the arm for pickup.

## Save GIF (`--save_gif`)

Saves the simulation as an animated GIF using offscreen rendering. Works with `--panda`, `--render`, and `--renderarm`. The GIF is written after the simulation completes.

**Requirements**: Display (DISPLAY) for OpenGL; ffmpeg (installed via visualiser dependencies).

### Example

```bash
# Panda AIF simulation → GIF
julia --project=. scripts/run_cli.jl --panda --speed 4 --robot 0.6 0 0.4 --init 0.4 0 0.2 --goal 0.6 0.2 0.35 --steps 100 --save_gif simulation.gif

# With renderarm
julia --project=. scripts/run_cli.jl --renderarm --goal 0.8 0.8 0.4 --init 0.3 0 0.2 --steps 500 --save_gif trajectory.gif
```

## Save MP4 (`--save_mp4`)

Saves the simulation as an H.264 MP4 video using offscreen rendering. Faster encoding than GIF with better quality. Uses `libx264` with `ultrafast` preset.

**Requirements**: Display (DISPLAY) for OpenGL; ffmpeg (installed via visualiser dependencies).

### Example

```bash
julia --project=. scripts/run_cli.jl --renderarm --goal 0.8 0.8 0.4 --init 0.3 0 0.2 --steps 500 --save_mp4 trajectory.mp4
```

## EMA Smoothing (`--alpha`)

Controls the Exponential Moving Average weight for control signal smoothing. At each step:

```
ctrl_smoothed = α × ctrl_new + (1 − α) × ctrl_previous
```

| `--alpha` value | Behaviour |
|-----------------|-----------|
| `0.15` | Ultra-smooth (slow response, very clean curve) |
| `0.3` | Default (balanced smoothness and responsiveness) |
| `0.5` | Moderate smoothing |
| `1.0` | No smoothing (raw discrete actions, may be jagged) |

## Playback Speed (`--speed`)

Controls the number of control steps per second during visualiser playback. Useful for slowing down the simulation so the arm movement is visible.

| `--speed` value | Behaviour |
|-----------------|-----------|
| `0` | No limit (default, runs as fast as possible) |
| `4` | 4 control steps/sec (viewable playback) |
| `10` | 10 control steps/sec (fast but visible) |

## Inference Backend (`--backend`)

Controls which belief-update engine is used during simulation:

- `analytic` (default): built-in closed-form diagonal-Gaussian Kalman update (`predict_belief!` + `update_belief!`)
- `rxinfer`: RxInfer.jl streaming message-passing filter (per-axis 1D linear-Gaussian state-space model)

Both produce mathematically identical posteriors. The RxInfer backend is useful for experimenting with richer probabilistic models (e.g. online noise learning) in the future.

### Example with RxInfer + Panda Arm

```bash
julia --project=. scripts/run_cli.jl --backend rxinfer --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 --steps 500 --ctrl_scale 3.0 --save_plot trajectory.png --renderarm
```

## Headless Fallback

When `--render` is set but no display is available (`DISPLAY` / `WAYLAND_DISPLAY` not set), the CLI automatically falls back to headless `run_simulation` and prints a notice.
