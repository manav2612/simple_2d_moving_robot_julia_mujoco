# MuJoCo models

## `models/robot.xml`

Main 3D robot scene:

- **World**: Ground plane, light, target site at (0.8, 0.8, 0.4)
- **Robot**: Body with three slide joints (x, y, z), sphere geom
- **Actuators**: Position actuators on slide_x, slide_y, slide_z, range ±10

## `models/scene.xml`

Alternative scene (legacy): free-floating robot body. `robot.xml` with slide joints is preferred for controlled 3D motion.
