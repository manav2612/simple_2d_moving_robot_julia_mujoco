# MuJoCo models

## `models/robot.xml`

Main 2D robot scene:

- **World**: Ground plane, light, target site at (0.8, 0.8)
- **Robot**: Body with two slide joints (x, y), sphere geom
- **Actuators**: Velocity motors on slide_x and slide_y, range ±1.5

## `models/scene.xml`

Alternative scene (legacy): free-floating robot body. `robot.xml` with slide joints is preferred for controlled 2D motion.
