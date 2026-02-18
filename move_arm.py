import mujoco
import mujoco.viewer
import numpy as np
import imageio
from pathlib import Path

# ---------------- Load model ----------------
ROOT = Path(__file__).resolve().parent
model = mujoco.MjModel.from_xml_path(str(ROOT / "panda_try.xml"))
data = mujoco.MjData(model)

# Offscreen renderer (FAST)
renderer = mujoco.Renderer(model, width=640, height=480)

# ---------------- Helpers ----------------
def _require_id(obj_type, name: str) -> int:
    idx = mujoco.mj_name2id(model, obj_type, name)
    if idx < 0:
        raise ValueError(f"Required MuJoCo name not found: {name!r} (type={obj_type})")
    return idx

# ---------------- IDs ----------------
ee_site_id = _require_id(mujoco.mjtObj.mjOBJ_SITE, "ee_center_site")
obj_site_id = _require_id(mujoco.mjtObj.mjOBJ_SITE, "obj_site")
target_site_id = _require_id(mujoco.mjtObj.mjOBJ_SITE, "target")

panda_mocap_body_id = _require_id(mujoco.mjtObj.mjOBJ_BODY, "panda_mocap")
mocap_id = int(model.body_mocapid[panda_mocap_body_id])
if mocap_id < 0:
    raise ValueError("Body 'panda_mocap' is not a mocap body (model.body_mocapid < 0).")

ee_center_body_id = _require_id(mujoco.mjtObj.mjOBJ_BODY, "ee_center_body")

# Use the named camera if present
try:
    _camera = "watching"
    renderer.update_scene(data, camera=_camera)
except TypeError:
    _camera = _require_id(mujoco.mjtObj.mjOBJ_CAMERA, "watching")

# ---------------- Init ----------------
mujoco.mj_forward(model, data)

# Initialize mocap to current end-effector pose (avoid big impulse at t=0)
data.mocap_pos[mocap_id] = data.site_xpos[ee_site_id].copy()
data.mocap_quat[mocap_id] = data.xquat[ee_center_body_id].copy()

# Initialize arm actuator targets to current joint positions (avoid fighting mocap)
for i in range(1, 8):
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator{i}")
    if act_id >= 0:
        joint_id = int(model.actuator_trnid[act_id, 0])
        qadr = int(model.jnt_qposadr[joint_id])
        data.ctrl[act_id] = float(data.qpos[qadr])

for _ in range(20):
    mujoco.mj_step(model, data)

# ---------------- Parameters ----------------
GAIN = 0.01
STOP_DIST = 0.03
GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.0
LIFT_HEIGHT = 0.25

phase = "reach"
frames = []
MAX_STEPS = 800

grip_r_id = _require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "r_gripper_finger_joint")
grip_l_id = _require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "l_gripper_finger_joint")

# ---------------- Main loop with on-screen viewer ----------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    for step in range(MAX_STEPS):
        ee_pos = data.site_xpos[ee_site_id].copy()
        obj_pos = data.site_xpos[obj_site_id].copy()
        target_pos = data.site_xpos[target_site_id].copy()

        if phase == "reach":
            approach = obj_pos + np.array([0.0, 0.0, 0.08])
            if np.linalg.norm(approach - ee_pos) > STOP_DIST:
                data.mocap_pos[mocap_id] += GAIN * (approach - ee_pos)
            else:
                phase = "grasp"

        elif phase == "grasp":
            data.ctrl[grip_r_id] = GRIPPER_CLOSED
            data.ctrl[grip_l_id] = GRIPPER_CLOSED
            hold_pos = data.mocap_pos[mocap_id].copy()
            phase = "lift"

        elif phase == "lift":
            lift_target = hold_pos + np.array([0.0, 0.0, LIFT_HEIGHT])
            if np.linalg.norm(lift_target - data.mocap_pos[mocap_id]) > STOP_DIST:
                data.mocap_pos[mocap_id] += GAIN * (lift_target - data.mocap_pos[mocap_id])
            else:
                phase = "move_to_target"

        elif phase == "move_to_target":
            place_target = target_pos + np.array([0.0, 0.0, 0.08])
            if np.linalg.norm(place_target - data.mocap_pos[mocap_id]) > STOP_DIST:
                data.mocap_pos[mocap_id] += GAIN * (place_target - data.mocap_pos[mocap_id])
            else:
                phase = "done"

        elif phase == "done":
            # Hold last pose for a few frames
            if step > MAX_STEPS - 40:
                pass

        # Step physics
        mujoco.mj_step(model, data)

        # Render to offscreen buffer and GIF
        renderer.update_scene(data, camera=_camera)
        frames.append(renderer.render())

        # Update on-screen viewer
        viewer.sync()

# ---------------- Save GIF ----------------
imageio.mimsave(
    str(ROOT / "full_motion_offscreen.gif"),
    frames,
    fps=30
)