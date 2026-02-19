# MuJoCo models

## `models/robot.xml`

Main 3D robot scene for AIF simulation:

- **World**: Ground plane, light, target site at (0.8, 0.8, 0.4). Zero gravity.
- **Robot**: Body `agent` with three slide joints (`slide_x`, `slide_y`, `slide_z`), sphere geom (`agent_body`). Each joint has `damping=15` for critically-damped response (no oscillation).
- **Actuators**: Position actuators (`px`, `py`, `pz`) on the three slide joints with `kp=80` (moderate gain for smooth tracking), `ctrlrange` ±10.

## `panda_render_scene.xml` (parent dir)

Scene for `--renderarm`, `--panda`, and `--save_gif`/`--save_mp4`: Panda arm with pick-and-place visuals.

- **Red object** (mocap): sphere at initial position. Stays at init until arm reaches it (pickup), then follows arm along trajectory (carry), then placed on top of green box (drop).
- **Green box**: static box at goal position (size 0.04).
- Includes `panda_mocap.xml` for the arm.
- Positions substituted at runtime from `--init` and `--goal` via `__INIT_POS__` / `__GOAL_POS__` placeholders.
- Camera `watching` provides a fixed viewpoint for GIF/MP4 recording.
- Trajectory: `[init_clamped, pos_1, pos_2, ...]` — arm first goes to init for pickup.

## `models/scene.xml`

Alternative scene (legacy): free-floating robot body. `robot.xml` with slide joints is preferred for controlled 3D motion.
