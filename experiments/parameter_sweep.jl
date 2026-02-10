#!/usr/bin/env julia
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using AIFMuJoCoRobot
using Printf

ctrls = [1.0, 2.5, 5.0]
obs_noises = [0.001, 0.01, 0.1]
process_noises = [0.001, 0.01, 0.1]
steps = 500
seed = 42

outpath = joinpath(@__DIR__, "sweep_results.csv")
open(outpath, "w") do io
    println(io, "ctrl_scale,obs_noise,process_noise,steps_used,converged,final_distance")
    for cs in ctrls, on in obs_noises, pn in process_noises
        @printf("Running: ctrl_scale=%.3f obs_noise=%.4f process_noise=%.4f\n", cs, on, pn)
        res = run_simulation(steps = steps, obs_noise = on, ctrl_scale = cs, process_noise = pn, seed = seed, verbose = false)
        pos = get_position(res.env)
        final_dist = sqrt(sum((pos .- [0.8,0.8]).^2))
        steps_used = length(res.history.positions)
        println(io, "$(cs),$(on),$(pn),$(steps_used),$(res.converged),$(round(final_dist, digits=6))")
    end
end

println("Parameter sweep finished. Results saved to: ", outpath)
