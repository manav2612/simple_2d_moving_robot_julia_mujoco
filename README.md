# AIF MuJoCo Robot

A **simple** Active Inference (AIF) controller for a 3D MuJoCo robot. The robot navigates toward a goal by minimizing Expected Free Energy (EFE).

## Overview

- **MuJoCo**: Physics simulation with a 3D point-mass robot (slide joints in x, y, z)
- **Active Inference**: Belief state, generative model, EFE-based policy selection
- **Julia**: Implemented in Julia using MuJoCo.jl

## Quick Start

```bash
cd simple_simple/aif_mujoco_robot
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. experiments/run_simulation.jl
```

## MuJoCo GUI Viewer

First install the visualiser (run once):

```bash
julia --project=. -e 'using MuJoCo; MuJoCo.install_visualiser()'
```

Then run the GUI:

```bash
julia --project=. scripts/visualize_mujoco.jl
```

Requires a display (WSLg or X11 on WSL2).

## Project Structure

| Path | Description |
|------|-------------|
| `src/AIFMuJoCoRobot.jl` | Main module |
| `src/aif/` | Active Inference (beliefs, EFE, policy, action) |
| `src/sim/` | MuJoCo env and sensors |
| `src/control/` | AIF controller |
| `models/robot.xml` | MuJoCo 2D robot model |
| `experiments/` | Configs and run script |
| `docs/` | Per-file documentation (MD) |

## Per-File Documentation

See the `docs/` folder for detailed explanations:

- `docs/utils_math.md` - Math utilities
- `docs/utils_logging.md` - Logging
- `docs/aif_beliefs.md` - Belief state
- `docs/aif_generative_model.md` - Generative model
- `docs/aif_efe.md` - Expected Free Energy
- `docs/aif_policy.md` - Policy selection
- `docs/aif_action.md` - Action representation
- `docs/sim_mujoco_env.md` - MuJoCo environment
- `docs/sim_sensors.md` - Sensors
- `docs/control_aif_controller.md` - AIF controller
- `docs/models.md` - MuJoCo models

## References

- [MuJoCo Julia Simulation](https://chatgpt.com/share/6987e046-7b38-8011-9584-054ec291c7c6)
- MuJoCo.jl: https://github.com/JamieMair/MuJoCo.jl

## MuJoCo usage

Prerequisites:

- Install MuJoCo and ensure the native library is findable (set `LD_LIBRARY_PATH` or OS equivalent).
- Place your MuJoCo license/key where MuJoCo expects it, or set `MUJOCO_KEY_PATH`.
- Activate the Julia project before running: `julia --project=.`

Wrapper script:

- A helper script `scripts/run_mujoco.sh` was added to invoke the CLI with passed arguments. Make it executable and run from the repo root:

```bash
chmod +x scripts/run_mujoco.sh
./scripts/run_mujoco.sh --goal 4.8 4.8 0.4 --init 2.5 2.5 0.2 --ctrl_scale 5.0 --steps 2000 --save_plot simulation.png --verbose true
```

Direct CLI usage:

```bash
julia --project=. scripts/run_cli.jl --goal 4.8 4.8 0.4 --init 2.5 2.5 0.2 --ctrl_scale 5.0 --steps 2000 --save_plot simulation.png --verbose true
```

Rendering and colors:

- To launch the MuJoCo visualiser during a run, pass `--render` to the CLI (requires a display):

```bash
julia --project=. scripts/run_cli.jl --render --goal 0.9 0.6 0.4 --init 0.5 0.5 0.2 --ctrl_scale 5.0 --steps 2000
```

- To change the robot (agent) color at runtime, pass `--agent_color r g b [a]` (three or four floats, 0..1). Example to make the agent red:

```bash
julia --project=. scripts/run_cli.jl --render --agent_color 1.0 0.0 0.0 --goal 0.9 0.6 0.4 --init 0.5 0.5 0.2 --steps 2000
```

- To change the goal marker color, pass `--goal_color r g b [a]` similarly. Example:

```bash
julia --project=. scripts/run_cli.jl --render --goal_color 0.0 1.0 0.0 --goal 0.9 0.6 0.4 --init 0.5 0.5 0.2 --steps 2000
```

Notes:

- `obs_noise` is treated as variance (σ²) in the code; default experiments use `0.01`.
- The MuJoCo model `models/robot.xml` uses position actuators — ensure `ctrlrange` covers your desired offsets (we use `-10 10` for large goals).
- If a long run shows no progress, confirm actuator ranges and try increasing `--ctrl_scale` (we found ~5.0 effective in sweeps).

Want this moved to a separate docs file or a CLI `--help`? I can add either.
