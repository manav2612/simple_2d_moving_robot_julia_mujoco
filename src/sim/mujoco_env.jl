"""
MuJoCo environment wrapper for the simple 2D robot.
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
function load_env(model_path::String; goal::Vector{Float64} = [0.8, 0.8])
    model = MuJoCo.load_model(model_path)
    data = MuJoCo.init_data(model)
    return EnvState(model, data, goal)
end

"""Reset environment to initial state. init_pos is world (x,y)."""
function reset!(env::EnvState; init_pos::Vector{Float64} = [-0.5, -0.5])
    MuJoCo.reset!(env.model, env.data)
    q = collect(env.data.qpos)
    q[1] = init_pos[1]
    q[2] = init_pos[2]
    env.data.qpos[:] = q
    return nothing
end

"""Step simulation. ctrl = desired displacement per step (from AIF policy)."""
function step!(env::EnvState, ctrl::AbstractVector; nsteps::Int = 5)
    pos = [env.data.qpos[1], env.data.qpos[2]]
    target = pos .+ ctrl
    # Default wide ctrlrange; attempt to read actuator ctrlrange from model
    min1, max1 = -10.0, 10.0
    min2, max2 = -10.0, 10.0
    try
        cr = getfield(env.model, :actuator_ctrlrange)
        if size(cr, 1) >= 2
            min1, max1 = cr[1,1], cr[1,2]
            min2, max2 = cr[2,1], cr[2,2]
        elseif size(cr, 1) == 1
            min1, max1 = cr[1,1], cr[1,2]
            min2, max2 = cr[1,1], cr[1,2]
        end
    catch
        # leave defaults
    end

    env.data.ctrl[1] = clamp(target[1], min1, max1)
    env.data.ctrl[2] = clamp(target[2], min2, max2)
    for _ in 1:nsteps
        MuJoCo.step!(env.model, env.data)
    end
    return nothing
end

"""Get current robot position [x, y]."""
function get_position(env::EnvState)
    return [env.data.qpos[1], env.data.qpos[2]]
end

"""Get goal position."""
function get_goal(env::EnvState)
    return copy(env.goal)
end

end
