"""
MuJoCo environment wrapper for the simple 3D robot.
"""
module MuJoCoEnv

using MuJoCo

export EnvState, load_env, reset!, step!, get_position, get_goal

struct EnvState
    model::MuJoCo.Model
    data::MuJoCo.Data
    goal::Vector{Float64}
end

"""Load MuJoCo model from path."""
function load_env(model_path::String; goal::Vector{Float64} = [0.8, 0.8, 0.4])
    model = MuJoCo.load_model(model_path)
    data = MuJoCo.init_data(model)
    return EnvState(model, data, goal)
end

"""Reset environment to initial state. init_pos is world (x, y, z)."""
function reset!(env::EnvState; init_pos::Vector{Float64} = [-0.5, -0.5, 0.2])
    MuJoCo.reset!(env.model, env.data)
    q = collect(env.data.qpos)
    q[1] = init_pos[1]
    q[2] = init_pos[2]
    q[3] = init_pos[3]
    env.data.qpos[:] = q
    return nothing
end

"""Step simulation. ctrl = desired displacement per step (from AIF policy)."""
function step!(env::EnvState, ctrl::AbstractVector; nsteps::Int = 5)
    pos = [env.data.qpos[1], env.data.qpos[2], env.data.qpos[3]]
    target = pos .+ ctrl
    # Default wide ctrlrange; attempt to read actuator ctrlrange from model
    mins = (-10.0, -10.0, -10.0)
    maxs = (10.0, 10.0, 10.0)
    try
        cr = getfield(env.model, :actuator_ctrlrange)
        nact = size(cr, 1)
        mins = ntuple(i -> i <= nact ? cr[i, 1] : -10.0, 3)
        maxs = ntuple(i -> i <= nact ? cr[i, 2] : 10.0, 3)
    catch
        # leave defaults
    end

    for i in 1:min(3, length(target))
        env.data.ctrl[i] = clamp(target[i], mins[i], maxs[i])
    end
    for _ in 1:nsteps
        MuJoCo.step!(env.model, env.data)
    end
    return nothing
end

"""Get current robot position [x, y, z]."""
function get_position(env::EnvState)
    return [env.data.qpos[1], env.data.qpos[2], env.data.qpos[3]]
end

"""Get goal position."""
function get_goal(env::EnvState)
    return copy(env.goal)
end

end
