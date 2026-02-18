"""
Panda try environment: box (obj) = object to pick, target site = red dot = place goal.
AIF drives the arm; move_arm.py logic for reach/grasp/lift/place.
"""
module PandaTryEnv

using MuJoCo

export PandaTryEnvState, load_panda_try_env

struct PandaTryEnvState
    model::MuJoCo.Model
    data::MuJoCo.Data
    robot_pos::Vector{Float64}
    init_pos::Vector{Float64}
    goal::Vector{Float64}
    mocap_row::Int
    ee_site_id::Int
    obj_site_id::Int
    target_site_id::Int
    obj_qposadr::Int
    grip_r_id::Int
    grip_l_id::Int
    mocap_gain::Float64
end

function _require_id(model::MuJoCo.Model, objtype, name::AbstractString)
    id = MuJoCo.mj_name2id(model, Cint(objtype), name)
    if id < 0
        error("Required MuJoCo object not found: name='$name', type=$objtype")
    end
    return id
end

"""Load panda_try.xml. obj=box to pick, target=red dot=place goal."""
function load_panda_try_env(
    path::String;
    robot_pos::Vector{Float64} = [0.6, 0.0, 0.4],
    init_pos::Vector{Float64} = [0.4, 0.0, 0.2],
    goal::Vector{Float64} = [0.6, 0.2, 0.35],
    mocap_gain::Float64 = 0.02,
)
    xml_str = read(path, String)
    init_str = join(string.(init_pos), " ")
    goal_str = join(string.(goal), " ")
    xml_str = replace(xml_str, "__INIT_POS__" => init_str, "__GOAL_POS__" => goal_str)
    scene_dir = dirname(path)
    tmp = joinpath(scene_dir, "panda_try_tmp.xml")
    write(tmp, xml_str)
    model = MuJoCo.load_model(tmp)
    rm(tmp; force = true)
    data = MuJoCo.init_data(model)
    MuJoCo.reset!(model, data)

    MuJoCo.forward!(model, data)
    for i in 1:7
        actname = "actuator$(i)"
        act_id = _require_id(model, MuJoCo.mjOBJ_ACTUATOR, actname)
        joint_id = Int(model.actuator_trnid[act_id + 1, 1])
        qadr = Int(model.jnt_qposadr[joint_id + 1])
        data.ctrl[act_id + 1] = data.qpos[qadr + 1]
    end

    panda_mocap_bid = _require_id(model, MuJoCo.mjOBJ_BODY, "panda_mocap")
    mocap_id = Int(model.body_mocapid[panda_mocap_bid + 1])
    mocap_id < 0 && error("Body 'panda_mocap' is not a mocap body")
    mocap_row = mocap_id + 1

    ee_site_id = _require_id(model, MuJoCo.mjOBJ_SITE, "ee_center_site")
    obj_site_id = _require_id(model, MuJoCo.mjOBJ_SITE, "obj_site")
    target_site_id = _require_id(model, MuJoCo.mjOBJ_SITE, "target")

    obj_joint_id = _require_id(model, MuJoCo.mjOBJ_JOINT, "obj_joint")
    obj_qposadr = Int(model.jnt_qposadr[obj_joint_id + 1])

    grip_r_id = _require_id(model, MuJoCo.mjOBJ_ACTUATOR, "r_gripper_finger_joint")
    grip_l_id = _require_id(model, MuJoCo.mjOBJ_ACTUATOR, "l_gripper_finger_joint")

    env = PandaTryEnvState(
        model, data,
        copy(robot_pos), copy(init_pos), copy(goal),
        mocap_row, ee_site_id, obj_site_id, target_site_id,
        obj_qposadr, grip_r_id, grip_l_id, mocap_gain,
    )
    PandaTryEnv.reset!(env; robot_pos = robot_pos, init_pos = init_pos, goal = goal)
    return env
end

function reset!(env::PandaTryEnvState;
    robot_pos::Union{Vector{Float64}, Nothing} = nothing,
    init_pos::Union{Vector{Float64}, Nothing} = nothing,
    goal::Union{Vector{Float64}, Nothing} = nothing,
)
    MuJoCo.reset!(env.model, env.data)
    if robot_pos !== nothing
        env.robot_pos .= robot_pos
    end
    if init_pos !== nothing
        env.init_pos .= init_pos
    end
    if goal !== nothing
        env.goal .= goal
    end

    @views for j in 1:3
        env.data.mocap_pos[env.mocap_row, j] = env.robot_pos[j]
    end
    # Set obj (free joint) position: qpos[obj_qposadr:obj_qposadr+6] = x,y,z, qw,qx,qy,qz
    for j in 1:3
        env.data.qpos[env.obj_qposadr + j] = env.init_pos[j]
    end
    env.data.qpos[env.obj_qposadr + 4] = 1.0  # qw
    for j in 5:7
        env.data.qpos[env.obj_qposadr + j] = 0.0
    end

    MuJoCo.forward!(env.model, env.data)
    return nothing
end

function get_position(env::PandaTryEnvState)
    @views return collect(env.data.site_xpos[env.ee_site_id + 1, :])
end

function get_goal(env::PandaTryEnvState)
    return copy(env.goal)
end

function get_obj_pos(env::PandaTryEnvState)
    @views return collect(env.data.site_xpos[env.obj_site_id + 1, :])
end

function get_target_pos(env::PandaTryEnvState)
    @views return collect(env.data.site_xpos[env.target_site_id + 1, :])
end

end
