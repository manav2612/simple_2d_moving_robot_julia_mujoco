#!/usr/bin/env julia
# Run the AIF MuJoCo robot simulation.
# Usage: julia --project=. experiments/run_simulation.jl
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
cd(joinpath(@__DIR__, ".."))
Pkg.instantiate()

using AIFMuJoCoRobot

cfg = AIFMuJoCoRobot.Configs.default_config()
result = run_simulation(;
    steps = cfg.steps,
    goal = cfg.goal,
    init_pos = cfg.init_pos,
    obs_noise = cfg.obs_noise,
    γ = cfg.γ,
    β = cfg.β,
    ctrl_scale = cfg.ctrl_scale,
    nsteps_per_ctrl = cfg.nsteps_per_ctrl,
    action_alpha = cfg.action_alpha,
    seed = cfg.seed,
    verbose = cfg.verbose,
)

println("Simulation finished. Converged: ", result.converged)
