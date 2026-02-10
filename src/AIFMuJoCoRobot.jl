"""
AIFMuJoCoRobot: Active Inference control for a simple 2D MuJoCo robot.
"""
module AIFMuJoCoRobot

using LinearAlgebra
using Random

# Utils
include("utils/math.jl")
include("utils/logging.jl")
using .MathUtils
using .LoggingUtils

# AIF submodules (order matters: Policy uses EFE, Beliefs)
include("aif/beliefs.jl")
include("aif/generative_model.jl")
include("aif/efe.jl")
include("aif/action.jl")
using .Beliefs
using .GenerativeModel
using .EFE
using .Action
include("aif/policy.jl")  # policy.jl uses EFE, Beliefs from parent scope
using .Policy

# Sim
include("sim/sensors.jl")
include("sim/mujoco_env.jl")
using .Sensors
using .MuJoCoEnv

# Control
include("control/aif_controller.jl")
using .AIFController

export run_simulation, default_model_path
export BeliefState, init_belief, update_belief!
export EnvState, load_env, reset!, step!, get_position, get_goal

# Configs (loaded from experiments)
const CONFIGS_PATH = normpath(joinpath(@__DIR__, "..", "experiments", "configs.jl"))
include(CONFIGS_PATH)
using .Configs

function default_model_path()
    return normpath(joinpath(@__DIR__, "..", "models", "robot.xml"))
end

"""Run the Active Inference + MuJoCo simulation."""
function run_simulation(; 
    steps::Int = 100,
    goal::Vector{Float64} = [0.8, 0.8],
    init_pos::Vector{Float64} = [-0.5, -0.5],
    obs_noise::Real = 0.01,
    γ::Real = 1.0,
    β::Real = 0.1,
    ctrl_scale::Real = 1.0,
    nsteps_per_ctrl::Int = 5,
    model_path::String = default_model_path(),
    seed::Union{Int,Nothing} = 42,
    process_noise::Real = 0.01,
    verbose::Bool = true
)
    seed !== nothing && Random.seed!(seed)
    rng = Random.default_rng()

    env = load_env(model_path; goal = goal)
    reset!(env; init_pos = init_pos)

    belief = init_belief(init_pos, [0.01, 0.01])
    history = (
        positions = Vector{Vector{Float64}}(),
        beliefs_mean = Vector{Vector{Float64}}(),
        actions = Vector{Vector{Float64}}(),
        efe_history = Float64[],
    )

    for t in 1:steps
        result = compute_control(belief, goal; γ = γ, β = β, ctrl_scale = ctrl_scale)
        ctrl, action, efe_val = result.ctrl, result.action, result.efe

        # Predict belief forward using chosen action (time/prior update)
        predict_belief!(belief, action; process_noise = process_noise)

        # Execute control in the MuJoCo environment
        step!(env, ctrl; nsteps = nsteps_per_ctrl)
        pos = get_position(env)
        obs = read_observation(collect(env.data.qpos); obs_noise = obs_noise, rng = rng)
        update_belief!(belief, obs; obs_noise = obs_noise)

        push!(history.positions, copy(pos))
        push!(history.beliefs_mean, copy(belief.mean))
        push!(history.actions, copy(action))
        push!(history.efe_history, efe_val)

        verbose && log_step(t, pos, goal, belief.mean, efe_val)

        dist = sqrt(sum((pos .- goal) .^ 2))
        if dist < 0.05
            verbose && log_summary(t, dist, true)
            return (env = env, belief = belief, history = history, converged = true)
        end
    end

    pos = get_position(env)
    dist = sqrt(sum((pos .- goal) .^ 2))
    verbose && log_summary(steps, dist, false)
    return (env = env, belief = belief, history = history, converged = false)
end

end
