# MuJoCo models

## `models/robot.xml`

Main 3D robot scene for AIF simulation:

- **World**: Ground plane, light, target site at (0.8, 0.8, 0.4)
- **Robot**: Body with three slide joints (x, y, z), sphere geom
- **Actuators**: Position actuators on slide_x, slide_y, slide_z, range ±10

## `panda_render_scene.xml` (parent dir)

Scene for `--renderarm`: Panda arm with pick-and-place visuals.

- **Red object** (mocap): sphere at initial position. Stays at init until arm reaches it (pickup), then follows arm along trajectory (carry), then placed on top of green box (drop).
- **Green box**: static at goal position
- Includes `panda_mocap.xml` for the arm
- Positions substituted at runtime from `--init` and `--goal`
- Trajectory: `[init_clamped, pos_1, pos_2, ...]` — arm first goes to init for pickup

## `models/scene.xml`

Alternative scene (legacy): free-floating robot body. `robot.xml` with slide joints is preferred for controlled 3D motion.
