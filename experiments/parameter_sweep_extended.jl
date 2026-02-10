#!/usr/bin/env julia
# Extended parameter sweep: varies `goal` and `init_pos` in addition to
# `ctrl_scale`, `obs_noise`, and `process_noise`.
#
# Edit the `goals`, `inits`, and other grids below to scale to any desired set.
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using AIFMuJoCoRobot
using Printf

# --- Editable grid (defaults chosen to keep run-time reasonable) ---
goals = [[0.8,0.8], [0.5, 0.2]]
inits = [[-0.5,-0.5], [-0.8, 0.0]]
ctrls = [2.5, 5.0]
obs_noises = [0.01, 0.1]
process_noises = [0.01]
steps = 500
seed = 42

outpath = joinpath(@__DIR__, "sweep_extended_results.csv")
open(outpath, "w") do io
    println(io, "goal_x,goal_y,init_x,init_y,ctrl_scale,obs_noise,process_noise,steps_used,converged,final_distance")
    for goal in goals, init_pos in inits, cs in ctrls, on in obs_noises, pn in process_noises
        @printf("Running: goal=(%.3f,%.3f) init=(%.3f,%.3f) ctrl=%.3f obs=%.4f pnoise=%.4f\n", goal[1], goal[2], init_pos[1], init_pos[2], cs, on, pn)
        res = run_simulation(steps = steps, goal = goal, init_pos = init_pos, obs_noise = on, ctrl_scale = cs, process_noise = pn, seed = seed, verbose = false)
        pos = get_position(res.env)
        final_dist = sqrt(sum((pos .- goal).^2))
        steps_used = length(res.history.positions)
        println(io, "$(goal[1]),$(goal[2]),$(init_pos[1]),$(init_pos[2]),$(cs),$(on),$(pn),$(steps_used),$(res.converged),$(round(final_dist, digits=6))")
    end
end

println("Extended sweep finished. Results saved to: ", outpath)
