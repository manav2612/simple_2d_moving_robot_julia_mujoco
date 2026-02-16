"""
AIFMuJoCoRobot: Active Inference control for a simple 3D MuJoCo robot.
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

# RxInfer streaming filter (optional backend)
include("aif/rxinfer_filter.jl")
using .RxInferFilter

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

"""Run the Active Inference + MuJoCo simulation.

Set `inference_backend = :rxinfer` to use the RxInfer streaming filter
instead of the built-in analytic diagonal-Gaussian updates (default `:analytic`).
"""
function run_simulation(; 
    steps::Int = 100,
    goal::Vector{Float64} = [0.8, 0.8, 0.4],
    init_pos::Vector{Float64} = [-0.5, -0.5, 0.2],
    obs_noise::Real = 0.01,
    γ::Real = 1.0,
    β::Real = 0.1,
    ctrl_scale::Real = 1.0,
    nsteps_per_ctrl::Int = 5,
    model_path::String = default_model_path(),
    seed::Union{Int,Nothing} = 42,
    process_noise = 0.005,
    verbose::Bool = true,
    inference_backend::Symbol = :analytic,
)
    seed !== nothing && Random.seed!(seed)
    rng = Random.default_rng()

    env = load_env(model_path; goal = goal)
    reset!(env; init_pos = init_pos)

    belief = init_belief(init_pos, [0.01, 0.01, 0.01])
    history = (
        positions = Vector{Vector{Float64}}(),
        beliefs_mean = Vector{Vector{Float64}}(),
        actions = Vector{Vector{Float64}}(),
        efe_history = Float64[],
    )

    # Optionally create the RxInfer streaming filter
    rxfilter = nothing
    if inference_backend == :rxinfer
        rxfilter = RxInferFilter.init_rxinfer_filter(
            init_pos, [0.01, 0.01, 0.01];
            obs_noise = obs_noise, process_noise = process_noise,
        )
    end

    try
        for t in 1:steps
            result = compute_control(belief, goal; γ = γ, β = β, ctrl_scale = ctrl_scale)
            ctrl, action, efe_val = result.ctrl, result.action, result.efe

            if inference_backend == :rxinfer
                # Predict + update via RxInfer streaming engine
                step!(env, ctrl; nsteps = nsteps_per_ctrl)
                pos = get_position(env)
                obs = read_observation(collect(env.data.qpos); obs_noise = obs_noise, rng = rng)
                post_mean, post_var = RxInferFilter.rxinfer_step!(rxfilter, ctrl, obs)
                belief.mean .= post_mean
                belief.cov  .= post_var
            else
                # Analytic diagonal-Gaussian predict + update (original path)
                predict_belief!(belief, ctrl; process_noise = process_noise)
                step!(env, ctrl; nsteps = nsteps_per_ctrl)
                pos = get_position(env)
                obs = read_observation(collect(env.data.qpos); obs_noise = obs_noise, rng = rng)
                update_belief!(belief, obs; obs_noise = obs_noise)
            end

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
    finally
        if rxfilter !== nothing
            RxInferFilter.rxinfer_stop!(rxfilter)
        end
    end

    pos = get_position(env)
    dist = sqrt(sum((pos .- goal) .^ 2))
    verbose && log_summary(steps, dist, false)
    return (env = env, belief = belief, history = history, converged = false)
end

end
