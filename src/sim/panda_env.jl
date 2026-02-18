"""
Panda arm environment adapter for AIF-driven control.
Maps AIF's 3D control to mocap updates for the Panda end-effector.
"""
module PandaEnv

using MuJoCo

export PandaEnvState, load_panda_env
# reset!, step!, get_position, get_goal not exported to avoid conflict with MuJoCoEnv

struct PandaEnvState
    model::MuJoCo.Model
    data::MuJoCo.Data
    robot_pos::Vector{Float64}
    init_pos::Vector{Float64}
    goal::Vector{Float64}
    mocap_row::Int
    red_mocap_row::Int
    ee_site_id::Int
    mocap_gain::Float64
end

function _require_id(model::MuJoCo.Model, objtype, name::AbstractString)
    id = MuJoCo.mj_name2id(model, Cint(objtype), name)
    if id < 0
        error("Required MuJoCo object not found: name='$name', type=$objtype")
    end
    return id
end

"""Load Panda scene from path. Substitutes __INIT_POS__ and __GOAL_POS__ in XML."""
function load_panda_env(
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
    # Write temp in same dir as source so include "panda_mocap.xml" resolves
    scene_dir = dirname(path)
    tmp = joinpath(scene_dir, "panda_render_scene_tmp.xml")
    write(tmp, xml_str)
    model = MuJoCo.load_model(tmp)
    rm(tmp; force = true)
    data = MuJoCo.init_data(model)
    MuJoCo.reset!(model, data)

    # Initialize arm actuators to current joint positions
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

    red_body_id = _require_id(model, MuJoCo.mjOBJ_BODY, "red_object")
    red_mocap_id = Int(model.body_mocapid[red_body_id + 1])
    red_mocap_id < 0 && error("Body 'red_object' is not a mocap body")
    red_mocap_row = red_mocap_id + 1

    ee_site_id = _require_id(model, MuJoCo.mjOBJ_SITE, "ee_center_site")

    env = PandaEnvState(
        model, data,
        copy(robot_pos), copy(init_pos), copy(goal),
        mocap_row, red_mocap_row, ee_site_id, mocap_gain,
    )
    reset!(env; robot_pos = robot_pos, init_pos = init_pos, goal = goal)
    return env
end

"""Reset environment. Sets mocap to robot_pos, red_object to init_pos."""
function reset!(env::PandaEnvState;
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

    # Set panda mocap (arm) to robot_pos
    @views for j in 1:3
        env.data.mocap_pos[env.mocap_row, j] = env.robot_pos[j]
    end
    # Set red object to init_pos
    @views for j in 1:3
        env.data.mocap_pos[env.red_mocap_row, j] = env.init_pos[j]
    end

    MuJoCo.forward!(env.model, env.data)
    return nothing
end

"""Get EE position from site (or mocap)."""
function get_position(env::PandaEnvState)
    @views return collect(env.data.site_xpos[env.ee_site_id + 1, :])
end

"""Get goal position."""
function get_goal(env::PandaEnvState)
    return copy(env.goal)
end

"""Step: interpolate mocap toward target = pos + ctrl, then physics step."""
function step!(env::PandaEnvState, ctrl::AbstractVector; nsteps::Int = 5)
    pos = get_position(env)
    target = pos .+ ctrl[1:3]
    mp = @views collect(env.data.mocap_pos[env.mocap_row, :])
    mp .+= env.mocap_gain .* (target .- mp)
    @views for j in 1:3
        env.data.mocap_pos[env.mocap_row, j] = mp[j]
    end
    for _ in 1:nsteps
        MuJoCo.step!(env.model, env.data)
    end
    return nothing
end

end
