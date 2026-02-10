#!/usr/bin/env julia
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using Printf

# Lightweight parameter sweep that launches the CLI with `--render` for each
# parameter combination. Each run writes a small log file (handled by the CLI),
# which we parse to produce a CSV summary. This is intended for manual /
# interactive inspection and is slower than headless sweeps.

goals = [[0.9, 0.7]]
inits = [[0.2, 0.5]]
ctrls = [5.0]
obs_noises = [0.01]
process_noises = [0.01]
steps = 5000
seed = 42

outpath = joinpath(@__DIR__, "sweep_render_results.csv")
open(outpath, "w") do io
    println(io, "goal_x,goal_y,init_x,init_y,ctrl_scale,obs_noise,process_noise,steps_used,converged,final_distance,log_path")
    for goal in goals, init_pos in inits, cs in ctrls, on in obs_noises, pn in process_noises
        @printf("Running (render): goal=(%.3f,%.3f) init=(%.3f,%.3f) ctrl=%.3f obs=%.4f pnoise=%.4f\n",
            goal[1], goal[2], init_pos[1], init_pos[2], cs, on, pn)

        save_plot = joinpath(@__DIR__, @sprintf("render_goal_%.3f_%.3f_init_%.3f_%.3f_ctrl_%.1f.png", goal[1], goal[2], init_pos[1], init_pos[2], cs))
        log_path = save_plot * ".log"

        cmd = `julia --project=. scripts/run_cli.jl --render --goal $(goal[1]) $(goal[2]) --init $(init_pos[1]) $(init_pos[2]) --ctrl_scale $(cs) --obs_noise $(on) --process_noise $(pn) --steps $(steps) --save_plot $(save_plot) --verbose false`
        try
            run(cmd)
        catch err
            @warn "CLI run failed" error=err
        end

        # read the per-run log produced by the CLI and extract final status
        converged = ""
        steps_used = ""
        final_dist = ""
        try
            txt = isfile(log_path) ? read(log_path, String) : ""
            m = match(r"Done\. Converged: (true|false), steps: (\d+), final_dist: ([0-9eE+\-.]+)", txt)
            if m !== nothing
                converged = m.captures[1]
                steps_used = m.captures[2]
                final_dist = m.captures[3]
            else
                # fallback: try to find last "Step" lines and estimate steps
                lines = split(txt, '\n')
                laststep = last(filter(x->startswith(strip(x), "[Step"), lines); default="")
                if laststep != ""
                    # crude parse: extract number inside [Step   N]
                    mm = match(r"\[Step\s+(\d+)\s*\]", laststep)
                    if mm !== nothing
                        steps_used = mm.captures[1]
                    end
                end
            end
        catch err
            @warn "Failed to read/parse log" log=log_path error=err
        end

        println(io, "$(goal[1]),$(goal[2]),$(init_pos[1]),$(init_pos[2]),$(cs),$(on),$(pn),$(steps_used),$(converged),$(final_dist),$(log_path)")
        flush(io)
    end
end

println("Render sweep finished. Results saved to: ", outpath)
