#!/usr/bin/env julia
# Simple CLI for ad-hoc runs of AIFMuJoCoRobot.run_simulation
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
cd(joinpath(@__DIR__, ".."))

using AIFMuJoCoRobot
using Printf
using MuJoCo
using Random
using Plots

# Global holder used by the MuJoCo visualiser controller to avoid closure/world-age issues
const MUJOCO_VIS_STATE = Ref{Any}(nothing)

function _mujoco_controller!(m, d)
    s = MUJOCO_VIS_STATE[]
    if s === nothing
        return nothing
    end

    belief = s[:belief]
    params = s[:params]
    rng = s[:rng]
    history = s[:history]

    pos = [d.qpos[1], d.qpos[2]]
    obs_noise = params[:obs_noise]
    # Initialize internal counters/state on first call
    if !haskey(s, :phys_step)
        s[:phys_step] = 0
        s[:last_ctrl] = [0.0, 0.0]
        s[:last_action] = [0.0, 0.0]
        s[:last_efe] = 0.0
    end

    s[:phys_step] += 1

    # control / belief update cadence
    nspc = params[:nsteps_per_ctrl]

    # Compute control at the start of each control block (matches run_simulation)
    compute_now = (nspc <= 1) || (s[:phys_step] % nspc == 1)
    if compute_now
        result = AIFMuJoCoRobot.AIFController.compute_control(
            belief,
            params[:goal];
            γ = params[:γ],
            β = params[:β],
            ctrl_scale = params[:ctrl_scale]
        )

        # Predict belief forward using the chosen action (matches run_simulation)
        try
            AIFMuJoCoRobot.predict_belief!(belief, result.action; process_noise = params[:process_noise])
        catch err
            @warn "predict_belief! failed in visualiser" error=err
        end

        s[:last_ctrl] = result.ctrl
        s[:last_action] = result.action
        s[:last_efe] = result.efe
    end

    # Apply the most recently computed control at every physics step
    target = pos .+ s[:last_ctrl]
    d.ctrl[1] = clamp(target[1], -10.0, 10.0)
    d.ctrl[2] = clamp(target[2], -10.0, 10.0)

    # After a full control block has been executed, read observation and update belief
    if nspc <= 1 || (s[:phys_step] % nspc == 0)
        obs = pos .+ (obs_noise > 0 ? sqrt(obs_noise) .* randn(rng, 2) : zeros(2))
        AIFMuJoCoRobot.update_belief!(belief, obs; obs_noise = obs_noise)

        # record history at control-step resolution (after execution)
        s[:step_counter] += 1
        push!(history[:positions], copy(pos))
        push!(history[:beliefs_mean], copy(belief.mean))
        push!(history[:actions], copy(s[:last_action]))
        push!(history[:efe_history], s[:last_efe])

        if s[:step_counter] >= params[:steps]
            println("Reached requested step count; exiting visualiser.")
            flush(stdout)
            exit(0)
        end
    end

    return nothing
end

function parse_float_pair(args, i)
    x = parse(Float64, args[i])
    y = parse(Float64, args[i+1])
    return (x, y), i+2
end

function parse_float_n(args, i, n)
    vals = Float64[]
    for j in 0:(n-1)
        push!(vals, parse(Float64, args[i+j]))
    end
    return vals, i + n
end

function parse_args()
    cfg = AIFMuJoCoRobot.Configs.default_config()
    params = Dict(
        :steps => cfg.steps,
        :goal => cfg.goal,
        :init_pos => cfg.init_pos,
        :obs_noise => cfg.obs_noise,
        :γ => cfg.γ,
        :β => cfg.β,
        :ctrl_scale => cfg.ctrl_scale,
        :nsteps_per_ctrl => cfg.nsteps_per_ctrl,
        :process_noise => 0.01,
        :seed => cfg.seed,
        :verbose => cfg.verbose,
        :save_plot => "",
        :render => false,
        :agent_color => nothing,
        :goal_color => nothing
    )

    args = copy(ARGS)
    i = 1
    while i <= length(args)
        a = args[i]
        if a == "--goal"
            (gx, gy), i = parse_float_pair(args, i+1)
            params[:goal] = [gx, gy]
        elseif a == "--init"
            (ix, iy), i = parse_float_pair(args, i+1)
            params[:init_pos] = [ix, iy]
        elseif a == "--obs_noise"
            params[:obs_noise] = parse(Float64, args[i+1]); i += 2
        elseif a == "--ctrl_scale"
            params[:ctrl_scale] = parse(Float64, args[i+1]); i += 2
        elseif a == "--process_noise"
            params[:process_noise] = parse(Float64, args[i+1]); i += 2
        elseif a == "--agent_color"
            # expects 3 or 4 floats: r g b [a]
            vals = Float64[]
            j = i + 1
            while j <= length(args) && length(vals) < 4 && !startswith(args[j], "--")
                push!(vals, parse(Float64, args[j]))
                j += 1
            end
            if length(vals) < 3
                error("--agent_color requires at least 3 float arguments: r g b [a]")
            end
            params[:agent_color] = vals
            i = j
        elseif a == "--goal_color"
            vals = Float64[]
            j = i + 1
            while j <= length(args) && length(vals) < 4 && !startswith(args[j], "--")
                push!(vals, parse(Float64, args[j]))
                j += 1
            end
            if length(vals) < 3
                error("--goal_color requires at least 3 float arguments: r g b [a]")
            end
            params[:goal_color] = vals
            i = j
        elseif a == "--steps"
            params[:steps] = parse(Int, args[i+1]); i += 2
        elseif a == "--seed"
            params[:seed] = parse(Int, args[i+1]); i += 2
        elseif a == "--verbose"
            v = lowercase(args[i+1]) in ("true","1","yes"); params[:verbose] = v; i += 2
        elseif a == "--save_plot"
            params[:save_plot] = args[i+1]; i += 2
        elseif a == "--render"
            # flag (no argument)
            params[:render] = true; i += 1
        else
            println("Unknown arg: ", a)
            i += 1
        end
    end
    return params
end

function main()
    params = parse_args()
    @printf("Running simulation with goal=(%.3f,%.3f) init=(%.3f,%.3f) ctrl_scale=%.3f obs_noise=%.4f steps=%d\n",
        params[:goal][1], params[:goal][2], params[:init_pos][1], params[:init_pos][2], params[:ctrl_scale], params[:obs_noise], params[:steps])

    # Open a small log file so messages are preserved when a visualiser window hijacks the terminal.
    log_path = params[:save_plot] != "" ? params[:save_plot] * ".log" : joinpath(@__DIR__, "run_cli.log")
    log_io = open(log_path, "a")
    # Expose result via a Ref so the atexit handler can attempt to save plots even on interrupt.
    RES = Ref{Any}(nothing)
    atexit() do
        try
            # Prefer RES[], but if visualiser was running and RES[] is empty, fall back to MUJOCO_VIS_STATE
            candidate = RES[]
            if candidate === nothing && MUJOCO_VIS_STATE[] !== nothing && haskey(MUJOCO_VIS_STATE[], :history)
                h = MUJOCO_VIS_STATE[][:history]
                candidate = (env = nothing, belief = nothing, history = (positions = h[:positions], beliefs_mean = h[:beliefs_mean], actions = h[:actions], efe_history = h[:efe_history]), converged = false)
            end

            if candidate !== nothing && params[:save_plot] != ""
                try
                    pos = candidate.history.positions
                    xs = [p[1] for p in pos]
                    ys = [p[2] for p in pos]
                    plt = plot(xs, ys, label = "Robot path", linewidth = 2, legend = :topright)
                    if length(xs) > 0
                        scatter!(plt, [xs[1]], [ys[1]], label = "Start", markersize = 8)
                    end
                    scatter!(plt, [params[:goal][1]], [params[:goal][2]], label = "Goal", markersize = 10)
                    xlabel!(plt, "x"); ylabel!(plt, "y"); title!(plt, "AIF Robot Trajectory")
                    savefig(plt, params[:save_plot])
                    try
                        println(log_io, "Saved plot to: " * params[:save_plot]); flush(log_io)
                    catch
                    end
                catch err
                    try
                        println(log_io, "Failed to save plot in atexit: " * string(err)); flush(log_io)
                    catch
                    end
                end
            else
                try
                    println(log_io, "No candidate history available to save plot."); flush(log_io)
                catch
                end
            end
        finally
            try
                close(log_io)
            catch
            end
        end
    end
    logmsg(args...) = begin
        s = join(string.(args), " ")
        println(s); flush(stdout)
        try
            println(log_io, s); flush(log_io)
        catch err
            # ignore logging errors
        end
    end

    # Run main logic; results are captured in `RES` and atexit will attempt to save plots/logs on exit
    res = nothing
        # Informative console log when render flag is provided so callers/tests can detect intent
        if params[:render]
            logmsg("Render flag enabled: attempting to use MuJoCo visualiser (render=true).")
        end

        if params[:render]
            # If no display is available, fall back to headless run_simulation
            if !(haskey(ENV, "DISPLAY") || haskey(ENV, "WAYLAND_DISPLAY"))
                logmsg("No DISPLAY detected, running headless simulation instead of visualiser.")
                res = run_simulation(
                    steps = params[:steps],
                    goal = params[:goal],
                    init_pos = params[:init_pos],
                    obs_noise = params[:obs_noise],
                    γ = params[:γ],
                    β = params[:β],
                    ctrl_scale = params[:ctrl_scale],
                    nsteps_per_ctrl = params[:nsteps_per_ctrl],
                    process_noise = params[:process_noise],
                    seed = params[:seed],
                    verbose = params[:verbose],
                )
            else
                # Launch MuJoCo visualiser with an embedded controller
                MuJoCo.init_visualiser()
                model_path = AIFMuJoCoRobot.default_model_path()
            # Allow overriding the agent geom color by creating a temporary model file
            model_path_to_load = model_path
            if params[:agent_color] !== nothing
                try
                    xml = read(model_path, String)
                    rgba_vals = params[:agent_color]
                    if length(rgba_vals) == 3
                        push!(rgba_vals, 1.0)
                    end
                    rgba_str = join(string.(rgba_vals), " ")
                    # replace rgba on geom with name="agent_body"
                    rx = Regex("""(<geom[^>]*name="agent_body"[^>]*rgba=")([^"]+)(")""")
                    if occursin(rx, xml)
                        newxml = replace(xml, rx) do m
                            return m.captures[1] * rgba_str * m.captures[3]
                        end
                    else
                        # fallback: try geom with name robot_geom
                        rx2 = Regex("""(<geom[^>]*name="robot_geom"[^>]*rgba=")([^"]+)(")""")
                        if occursin(rx2, xml)
                            newxml = replace(xml, rx2) do m
                                return m.captures[1] * rgba_str * m.captures[3]
                            end
                        else
                            newxml = xml
                        end
                    end
                    # optionally replace target/site color for the goal marker
                    if params[:goal_color] !== nothing
                        gc = params[:goal_color]
                        if length(gc) == 3
                            push!(gc, 1.0)
                        end
                        gstr = join(string.(gc), " ")
                        rxg = Regex("""(<site[^>]*name="target"[^>]*rgba=")([^"]+)(")""")
                        if occursin(rxg, newxml)
                            newxml = replace(newxml, rxg) do m
                                return m.captures[1] * gstr * m.captures[3]
                            end
                        end
                    end
                    tmp = tempname() * ".xml"
                    open(tmp, "w") do io
                        write(io, newxml)
                    end
                    model_path_to_load = tmp
                catch err
                    @warn "Failed to create temporary model with custom color: $err"
                    model_path_to_load = model_path
                end
            end

            model = MuJoCo.load_model(model_path_to_load)
            data = MuJoCo.init_data(model)
            MuJoCo.reset!(model, data)

            # set initial position
            q = collect(data.qpos)
            q[1] = params[:init_pos][1]
            q[2] = params[:init_pos][2]
            data.qpos[:] = q

            Random.seed!(params[:seed])
            rng = Random.default_rng()

            belief = AIFMuJoCoRobot.init_belief(params[:init_pos], [0.01, 0.01])
            obs_noise = params[:obs_noise]
            ctrl_scale = params[:ctrl_scale]

            # populate global visualiser state and call the top-level controller
            MUJOCO_VIS_STATE[] = Dict(
                :belief => belief,
                :params => params,
                :rng => rng,
                :step_counter => 0,
                :history => Dict(
                    :positions => Vector{Vector{Float64}}(),
                    :beliefs_mean => Vector{Vector{Float64}}(),
                    :actions => Vector{Vector{Float64}}(),
                    :efe_history => Float64[],
                ),
            )

                logmsg("Launching MuJoCo visualizer... Close window to exit or it will auto-exit after steps.")
                flush(stdout)
                vis_state = nothing
                # Run the visualiser in an async task so the CLI can continue printing live updates.
                vis_task = @async begin
                    try
                        Base.invokelatest(MuJoCo.visualise!, model, data; controller = _mujoco_controller!)
                    catch err
                        @warn "MuJoCo visualiser exited with error" error=err
                    end
                end

                # Poll the shared visualiser state and print live updates while the visualiser runs
                last_step = 0
                while true
                    sleep(0.05)
                    s = MUJOCO_VIS_STATE[]
                    if s === nothing
                        # visualiser not yet initialised
                        continue
                    end
                    hist = s[:history]
                    step = s[:step_counter]
                    if step > last_step && length(hist[:positions]) >= 1
                        pos = hist[:positions][end]
                        dist = sqrt(sum((pos .- params[:goal]).^2))
                        logmsg("[Step ", step, "] pos=(", round(pos[1],digits=3), ", ", round(pos[2],digits=3), ") goal=(", params[:goal][1], ", ", params[:goal][2], ") dist=", round(dist,digits=4))
                        last_step = step
                    end
                    # exit when requested steps reached or visualiser task finished
                    if step >= params[:steps] || istaskdone(vis_task)
                        break
                    end
                end

                # capture and clear visualiser state
                vis_state = MUJOCO_VIS_STATE[]
                MUJOCO_VIS_STATE[] = nothing

            # Extract history from visualiser state if present
            hist = (positions = Vector{Vector{Float64}}(), beliefs_mean = Vector{Vector{Float64}}(), actions = Vector{Vector{Float64}}(), efe_history = Float64[])
            if vis_state !== nothing && haskey(vis_state, :history)
                h = vis_state[:history]
                hist = (positions = h[:positions], beliefs_mean = h[:beliefs_mean], actions = h[:actions], efe_history = h[:efe_history])
            end

            # Return an EnvState so downstream logic (e.g., get_position) can safely query the environment
            envstate = nothing
            try
                envstate = AIFMuJoCoRobot.MuJoCoEnv.EnvState(model, data, params[:goal])
            catch err
                @warn "Failed to construct EnvState from model/data" error=err
                envstate = nothing
            end

            res = (env = envstate, belief = belief, history = hist, converged = false)
            RES[] = res
        end
    else
        res = run_simulation(
            steps = params[:steps],
            goal = params[:goal],
            init_pos = params[:init_pos],
            obs_noise = params[:obs_noise],
            γ = params[:γ],
            β = params[:β],
            ctrl_scale = params[:ctrl_scale],
            nsteps_per_ctrl = params[:nsteps_per_ctrl],
            process_noise = params[:process_noise],
            seed = params[:seed],
            verbose = params[:verbose],
        )
        RES[] = res
    end

    # Ensure `res.env` is a valid EnvState so downstream calls (e.g., `get_position`) succeed.
    if res.env === nothing
        try
            env = AIFMuJoCoRobot.load_env(AIFMuJoCoRobot.default_model_path(); goal = params[:goal])
            AIFMuJoCoRobot.reset!(env; init_pos = params[:init_pos])
            res = (env = env, belief = res.belief, history = res.history, converged = res.converged)
        catch err
            @warn "Failed to construct fallback EnvState after visualiser: $err"
        end
    end

    pos = get_position(res.env)
    final_dist = sqrt(sum((pos .- params[:goal]).^2))
    try
        println("Done. Converged: ", res.converged, ", steps: ", length(res.history.positions), ", final_dist: ", round(final_dist,digits=6))
        flush(stdout)
        try
            println(log_io, "Done. Converged: " * string(res.converged) * ", steps: " * string(length(res.history.positions)) * ", final_dist: " * string(round(final_dist,digits=6))); flush(log_io)
        catch
        end
    catch
    end

    if params[:save_plot] != ""
        # Simple inline plotting to avoid include / method scope issues
        try
            pos = res.history.positions
            xs = [p[1] for p in pos]
            ys = [p[2] for p in pos]
            plt = plot(xs, ys, label = "Robot path", linewidth = 2, legend = :topright)
            scatter!(plt, [xs[1]], [ys[1]], label = "Start", markersize = 8)
            scatter!(plt, [params[:goal][1]], [params[:goal][2]], label = "Goal", markersize = 10)
            xlabel!(plt, "x"); ylabel!(plt, "y"); title!(plt, "AIF Robot Trajectory")
            savefig(plt, params[:save_plot])
            try
                println("Saved plot to: ", params[:save_plot])
                flush(stdout)
                println(log_io, "Saved plot to: " * params[:save_plot]); flush(log_io)
            catch
            end
        catch err
            try
                println("Failed to save plot: ", err)
                flush(stdout)
                println(log_io, "Failed to save plot: " * string(err)); flush(log_io)
            catch
            end
        end
    end
end

main()
