# Plot trajectories from simulation results.
# Usage: include("plots/trajectories.jl") after run_simulation.

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using Plots

function plot_trajectory(result; goal = [0.8, 0.8], save_path = nothing)
    pos = result.history.positions
    xs = [p[1] for p in pos]
    ys = [p[2] for p in pos]
    plot(xs, ys, label = "Robot path", linewidth = 2, legend = :topright)
    scatter!([xs[1]], [ys[1]], label = "Start", markersize = 8)
    scatter!([goal[1]], [goal[2]], label = "Goal", markersize = 10)
    xlabel!("x")
    ylabel!("y")
    title!("AIF Robot Trajectory")
    save_path !== nothing && savefig(save_path)
    return current()
end
