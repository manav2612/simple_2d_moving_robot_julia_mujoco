#!/usr/bin/env julia
# Move Panda arm (reach, grasp, lift, place) using MuJoCo.jl
# Run from project root:
#   julia --project=aif_mujoco_robot aif_mujoco_robot/scripts/move_panda_arm.jl

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.instantiate()

using MuJoCo
using LinearAlgebra

MuJoCo.init_visualiser()

# ---------------- Load model ----------------
# panda_try.xml lives one level above the Julia project root
panda_xml = normpath(joinpath(@__DIR__, "..", "..", "panda_try.xml"))
@info "Loading model" panda_xml

model = MuJoCo.load_model(panda_xml)
data  = MuJoCo.init_data(model)
MuJoCo.reset!(model, data)

# ---------------- Helpers ----------------
function require_id(model::MuJoCo.Model, objtype, name::AbstractString)
    # Use low-level MuJoCo C API (0-based ids, Cint type)
    id = MuJoCo.mj_name2id(model, Cint(objtype), name)
    if id < 0
        error("Required MuJoCo object not found: name='$name', type=$objtype")
    end
    return id
end

# ---------------- IDs ----------------
const EE_SITE_ID      = require_id(model, MuJoCo.mjOBJ_SITE, "ee_center_site")
const OBJ_SITE_ID     = require_id(model, MuJoCo.mjOBJ_SITE, "obj_site")
const TARGET_SITE_ID  = require_id(model, MuJoCo.mjOBJ_SITE, "target")

const PANDA_MOCAP_BID = require_id(model, MuJoCo.mjOBJ_BODY, "panda_mocap")
const EE_BODY_ID      = require_id(model, MuJoCo.mjOBJ_BODY, "ee_center_body")

const MOCAP_ID = Int(model.body_mocapid[PANDA_MOCAP_BID + 1])
MOCAP_ID < 0 && error("Body 'panda_mocap' is not a mocap body (body_mocapid < 0)")
# MuJoCo.jl exposes mocap_pos as a 3×nmocap array; convert 0-based mocap id to Julia column index.
const MOCAP_COL = MOCAP_ID + 1

const GRIP_R_ID = require_id(model, MuJoCo.mjOBJ_ACTUATOR, "r_gripper_finger_joint")
const GRIP_L_ID = require_id(model, MuJoCo.mjOBJ_ACTUATOR, "l_gripper_finger_joint")

# ---------------- Init ----------------
# Initialize arm actuator targets to current joint positions
MuJoCo.forward!(model, data)
for i in 1:7
    actname = "actuator$(i)"
    act_id = require_id(model, MuJoCo.mjOBJ_ACTUATOR, actname)  # 0-based actuator id
    # Map actuator -> joint -> qpos index (0-based indices from MuJoCo)
    joint_id = Int(model.actuator_trnid[act_id + 1, 1])
    qadr     = Int(model.jnt_qposadr[joint_id + 1])
    data.ctrl[act_id + 1] = data.qpos[qadr + 1]
end

# ---------------- Parameters ----------------
const GAIN           = 0.01
const STOP_DIST      = 0.03
const GRIPPER_OPEN   = 0.04
const GRIPPER_CLOSED = 0.0
const LIFT_HEIGHT    = 0.25

const phase = Ref("reach")
hold_pos = zeros(3)

function site_pos(d::MuJoCo.Data, sid::Integer)
    @views return collect(d.site_xpos[sid + 1, :])
end

function step_phase!(m::MuJoCo.Model, d::MuJoCo.Data)
    ee_pos     = site_pos(d, EE_SITE_ID)
    obj_pos    = site_pos(d, OBJ_SITE_ID)
    target_pos = site_pos(d, TARGET_SITE_ID)

    # Work on a local copy of mocap position (row = mocap body, columns = x,y,z),
    # then write it back at the end.
    mp = @views collect(d.mocap_pos[MOCAP_COL, :])

    if phase[] == "reach"
        approach = obj_pos .+ [0.0, 0.0, 0.08]
        if norm(approach - ee_pos) > STOP_DIST
            mp .+= GAIN .* (approach .- ee_pos)
        else
            phase[] = "grasp"
        end

    elseif phase[] == "grasp"
        d.ctrl[GRIP_R_ID + 1] = GRIPPER_CLOSED
        d.ctrl[GRIP_L_ID + 1] = GRIPPER_CLOSED
        hold_pos .= mp
        phase[] = "lift"

    elseif phase[] == "lift"
        lift_target = hold_pos .+ [0.0, 0.0, LIFT_HEIGHT]
        if norm(lift_target - mp) > STOP_DIST
            mp .+= GAIN .* (lift_target .- mp)
        else
            phase[] = "move_to_target"
        end

    elseif phase[] == "move_to_target"
        place_target = target_pos .+ [0.0, 0.0, 0.08]
        if norm(place_target - mp) > STOP_DIST
            mp .+= GAIN .* (place_target .- mp)
        else
            phase[] = "done"
        end

    elseif phase[] == "done"
        # Just hold pose
    end

    # Write updated mocap pos back into Data (component-wise: row = mocap body, cols = x,y,z)
    @views for j in 1:3
        d.mocap_pos[MOCAP_COL, j] = mp[j]
    end
end

# Controller called by MuJoCo.visualise! each simulation step
function controller!(m::MuJoCo.Model, d::MuJoCo.Data)
    step_phase!(m, d)
    return nothing
end

println("Launching MuJoCo visualizer for Panda pick-and-place. Close window to exit.")
MuJoCo.visualise!(model, data, controller = controller!)

