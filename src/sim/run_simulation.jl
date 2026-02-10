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
    steps = 500,
    goal = cfg.goal,
    init_pos = cfg.init_pos,
    obs_noise = cfg.obs_noise,
    γ = cfg.γ,
    β = cfg.β,
    ctrl_scale = 1.0,
    nsteps_per_ctrl = cfg.nsteps_per_ctrl,
    seed = cfg.seed,
    verbose = cfg.verbose,
)

println("Simulation finished. Converged: ", result.converged)

# Plot and save the trajectory
include(joinpath(@__DIR__, "..", "plots", "trajectories.jl"))
plot_trajectory(result; goal=cfg.goal, save_path="simulation_trajectory.png")
println("Trajectory plot saved to simulation_trajectory.png")
