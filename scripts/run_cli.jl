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

# Workspace bounds chosen to stay inside Panda arm's reachable region.
const PANDA_X_MIN = 0.2
const PANDA_X_MAX = 0.8
const PANDA_Y_MIN = -0.4
const PANDA_Y_MAX = 0.4
const PANDA_Z_MIN = 0.1
const PANDA_Z_MAX = 0.5

function _require_id_local(model::MuJoCo.Model, objtype, name::AbstractString)
    id = MuJoCo.mj_name2id(model, Cint(objtype), name)
    if id < 0
        error("Required MuJoCo object not found: name='$name', type=$objtype")
    end
    return id
end

function render_panda_arm_from_history(history, params)
    positions = getfield(history, :positions)
    if length(positions) == 0
        println("No positions in history; skipping Panda arm render.")
        flush(stdout)
        return
    end

    # Scene: red object at init_pos, green box at goal. After trajectory, red on top of box.
    init_pos = params[:init_pos]
    goal = params[:goal]
    init_str = @sprintf("%.4f %.4f %.4f", init_pos[1], init_pos[2], init_pos[3])
    goal_str = @sprintf("%.4f %.4f %.4f", goal[1], goal[2], goal[3])

    MuJoCo.init_visualiser()
    scene_dir = normpath(joinpath(@__DIR__, "..", ".."))
    scene_path = joinpath(scene_dir, "panda_render_scene.xml")
    xml_str = read(scene_path, String)
    xml_str = replace(xml_str, "__INIT_POS__" => init_str, "__GOAL_POS__" => goal_str)
    tmp_xml = joinpath(scene_dir, "panda_render_scene_tmp.xml")
    write(tmp_xml, xml_str)

    @printf("Rendering Panda arm: red at init=(%.2f,%.2f,%.2f), green box at goal=(%.2f,%.2f,%.2f)\n",
            init_pos[1], init_pos[2], init_pos[3], goal[1], goal[2], goal[3])
    @printf("Trajectory: %d AIF steps\n", length(positions))
    flush(stdout)

    model = MuJoCo.load_model(tmp_xml)
    rm(tmp_xml; force = true)
    data  = MuJoCo.init_data(model)
    MuJoCo.reset!(model, data)

    MuJoCo.forward!(model, data)
    for i in 1:7
        actname = "actuator$(i)"
        act_id = _require_id_local(model, MuJoCo.mjOBJ_ACTUATOR, actname)
        joint_id = Int(model.actuator_trnid[act_id + 1, 1])
        qadr     = Int(model.jnt_qposadr[joint_id + 1])
        data.ctrl[act_id + 1] = data.qpos[qadr + 1]
    end

    panda_mocap_body_id = _require_id_local(model, MuJoCo.mjOBJ_BODY, "panda_mocap")
    mocap_id = Int(model.body_mocapid[panda_mocap_body_id + 1])
    mocap_id < 0 && error("Body 'panda_mocap' is not a mocap body (body_mocapid < 0)")
    mocap_row = mocap_id + 1

    red_body_id = _require_id_local(model, MuJoCo.mjOBJ_BODY, "red_object")
    red_mocap_id = Int(model.body_mocapid[red_body_id + 1])
    red_mocap_id < 0 && error("Body 'red_object' is not a mocap body")
    red_mocap_row = red_mocap_id + 1

    gain = 0.02
    stop_dist = 0.02

    # Hardcoded final position: red object on top of green box (box half-size 0.04, sphere radius 0.025)
    box_half = 0.04
    sphere_r = 0.025
    red_on_box = [goal[1], goal[2], goal[3] + box_half + sphere_r]

    # Prepend init (clamped) so arm first goes to red ball for pickup, then follows AIF path
    init_clamped = [clamp(init_pos[1], PANDA_X_MIN, PANDA_X_MAX),
                   clamp(init_pos[2], PANDA_Y_MIN, PANDA_Y_MAX),
                   clamp(init_pos[3], PANDA_Z_MIN, PANDA_Z_MAX)]
    panda_traj = Vector{Vector{Float64}}(undef, length(positions) + 1)
    panda_traj[1] = init_clamped
    for (i, p) in enumerate(positions)
        x = clamp(p[1], PANDA_X_MIN, PANDA_X_MAX)
        y = clamp(p[2], PANDA_Y_MIN, PANDA_Y_MAX)
        z = clamp(p[3], PANDA_Z_MIN, PANDA_Z_MAX)
        panda_traj[i + 1] = [x, y, z]
    end

    first_xyz = panda_traj[1]
    @views begin
        data.mocap_pos[mocap_row, 1] = first_xyz[1]
        data.mocap_pos[mocap_row, 2] = first_xyz[2]
        data.mocap_pos[mocap_row, 3] = first_xyz[3]
    end

    PANDA_VIS_STATE[] = Dict(
        :traj => panda_traj,
        :idx => 1,
        :gain => gain,
        :stop_dist => stop_dist,
        :mocap_row => mocap_row,
        :red_mocap_row => red_mocap_row,
        :red_on_box => red_on_box,
        :place_gain => 0.03,
        :vis_step => 0,
        :last_logged_idx => 0,
    )

    function _panda_controller!(m, d)
        s = PANDA_VIS_STATE[]
        s === nothing && return

        traj = s[:traj]
        idx = s[:idx]
        s[:vis_step] += 1

        # Panda arm: follow AIF trajectory
        if idx <= length(traj)
            pos3d = traj[idx]
            target = [pos3d[1], pos3d[2], pos3d[3]]
            r = s[:mocap_row]
            mp = @views collect(d.mocap_pos[r, :])
            dx = target .- mp
            if sqrt(sum(dx.^2)) > s[:stop_dist]
                mp .+= s[:gain] .* dx
            else
                s[:idx] = idx + 1
                @printf("Panda idx %d/%d | target=(%.3f, %.3f, %.3f)\n",
                        idx, length(traj), target[1], target[2], target[3])
                flush(stdout)
            end
            @views for j in 1:3
                d.mocap_pos[r, j] = mp[j]
            end
            # Pickup + carry: red ball follows arm only after arm reaches init (idx>=2)
            if idx >= 2
                rr = s[:red_mocap_row]
                @views for j in 1:3
                    d.mocap_pos[rr, j] = mp[j]
                end
            end
        else
            # Trajectory done: place red object on top of green box (hardcoded grasper)
            rr = s[:red_mocap_row]
            rp = @views collect(d.mocap_pos[rr, :])
            dest = s[:red_on_box]
            dr = dest .- rp
            if sqrt(sum(dr.^2)) > 0.005
                rp .+= s[:place_gain] .* dr
                @views for j in 1:3
                    d.mocap_pos[rr, j] = rp[j]
                end
            end
        end

        PANDA_VIS_STATE[] = s
        return nothing
    end

    println("Launching Panda arm visualiser. Red at init, green box at goal. After trajectory, red placed on box.")
    flush(stdout)

    Base.invokelatest(MuJoCo.visualise!, model, data; controller = _panda_controller!)
end

# Global holders used by MuJoCo visualisers to avoid closure/world-age issues
const MUJOCO_VIS_STATE = Ref{Any}(nothing)
const PANDA_VIS_STATE  = Ref{Any}(nothing)

function _mujoco_controller!(m, d)
    s = MUJOCO_VIS_STATE[]
    if s === nothing
        return nothing
    end

    belief = s[:belief]
    params = s[:params]
    rng = s[:rng]
    history = s[:history]

    pos = [d.qpos[1], d.qpos[2], d.qpos[3]]
    obs_noise = params[:obs_noise]
    # Initialize internal counters/state on first call
    if !haskey(s, :phys_step)
        s[:phys_step] = 0
        s[:last_ctrl] = [0.0, 0.0, 0.0]
        s[:last_action] = [0.0, 0.0, 0.0]
        s[:last_efe] = 0.0
        s[:prev_smooth_ctrl] = [0.0, 0.0, 0.0]
        s[:ctrl_step_count] = 0
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

        # EMA smoothing of control signals for smooth trajectories
        α_smooth = get(params, :action_alpha, 0.3)
        s[:ctrl_step_count] += 1
        raw_ctrl = result.ctrl
        if s[:ctrl_step_count] <= 1
            smoothed = copy(raw_ctrl)
        else
            smoothed = α_smooth .* raw_ctrl .+ (1.0 - α_smooth) .* s[:prev_smooth_ctrl]
        end
        s[:prev_smooth_ctrl] = copy(smoothed)

        # Predict belief using smoothed control
        try
            AIFMuJoCoRobot.predict_belief!(belief, smoothed; process_noise = params[:process_noise])
        catch err
            @warn "predict_belief! failed in visualiser" error=err
        end

        s[:last_ctrl] = smoothed
        s[:last_action] = result.action
        s[:last_efe] = result.efe
    end

    # Apply the most recently computed control at every physics step
    target = pos .+ s[:last_ctrl]
    d.ctrl[1] = clamp(target[1], -10.0, 10.0)
    d.ctrl[2] = clamp(target[2], -10.0, 10.0)
    d.ctrl[3] = clamp(target[3], -10.0, 10.0)

    # After a full control block has been executed, read observation and update belief
    if nspc <= 1 || (s[:phys_step] % nspc == 0)
        obs = pos .+ (obs_noise > 0 ? sqrt(obs_noise) .* randn(rng, 3) : zeros(3))
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

"""Parse three floats (x, y, z) for 3D goal/init."""
function parse_float_triple(args, i)
    x = parse(Float64, args[i])
    y = parse(Float64, args[i+1])
    z = parse(Float64, args[i+2])
    return [x, y, z], i+3
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
        :process_noise => cfg.process_noise,
        :action_alpha => cfg.action_alpha,
        :seed => cfg.seed,
        :verbose => cfg.verbose,
        :save_plot => "",
        :render => false,
        :renderarm => false,
        :agent_color => nothing,
        :goal_color => nothing,
        :inference_backend => cfg.inference_backend
    )

    args = copy(ARGS)
    i = 1
    while i <= length(args)
        a = args[i]
        if a == "--goal"
            params[:goal], i = parse_float_triple(args, i+1)
        elseif a == "--init"
            params[:init_pos], i = parse_float_triple(args, i+1)
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
        elseif a == "--renderarm"
            # 3D Panda arm visualisation using AIF 2D trajectory (flag, no argument)
            params[:renderarm] = true; i += 1
        elseif a == "--backend"
            # Inference backend: "analytic" (default) or "rxinfer"
            params[:inference_backend] = Symbol(args[i+1]); i += 2
        elseif a == "--alpha"
            # EMA smoothing weight (0-1): lower = smoother trajectory
            params[:action_alpha] = parse(Float64, args[i+1]); i += 2
        else
            println("Unknown arg: ", a)
            i += 1
        end
    end
    return params
end

function main()
    params = parse_args()
    @printf("Running simulation with goal=(%.3f,%.3f,%.3f) init=(%.3f,%.3f,%.3f) ctrl_scale=%.3f obs_noise=%.4f alpha=%.2f steps=%d\n",
        params[:goal][1], params[:goal][2], params[:goal][3], params[:init_pos][1], params[:init_pos][2], params[:init_pos][3], params[:ctrl_scale], params[:obs_noise], params[:action_alpha], params[:steps])

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
                    zs = [p[3] for p in pos]
                    plt = plot(xs, ys, zs, label = "Robot path", linewidth = 2, legend = :topright)
                    if length(xs) > 0
                        scatter!(plt, [xs[1]], [ys[1]], [zs[1]], label = "Start", markersize = 8)
                    end
                    scatter!(plt, [params[:goal][1]], [params[:goal][2]], [params[:goal][3]], label = "Goal", markersize = 10)
                    xlabel!(plt, "x"); ylabel!(plt, "y"); zlabel!(plt, "z"); title!(plt, "AIF Robot Trajectory (3D)")
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
                    inference_backend = params[:inference_backend],
                    action_alpha = params[:action_alpha],
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

            # set initial position (x, y, z)
            q = collect(data.qpos)
            q[1] = params[:init_pos][1]
            q[2] = params[:init_pos][2]
            q[3] = params[:init_pos][3]
            data.qpos[:] = q

            Random.seed!(params[:seed])
            rng = Random.default_rng()

            belief = AIFMuJoCoRobot.init_belief(params[:init_pos], [0.01, 0.01, 0.01])
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
                        logmsg("[Step ", step, "] pos=(", round(pos[1],digits=3), ", ", round(pos[2],digits=3), ", ", round(pos[3],digits=3), ") goal=(", params[:goal][1], ", ", params[:goal][2], ", ", params[:goal][3], ") dist=", round(dist,digits=4))
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
            inference_backend = params[:inference_backend],
            action_alpha = params[:action_alpha],
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
            zs = [p[3] for p in pos]
            plt = plot(xs, ys, zs, label = "Robot path", linewidth = 2, legend = :topright)
            scatter!(plt, [xs[1]], [ys[1]], [zs[1]], label = "Start", markersize = 8)
            scatter!(plt, [params[:goal][1]], [params[:goal][2]], [params[:goal][3]], label = "Goal", markersize = 10)
            xlabel!(plt, "x"); ylabel!(plt, "y"); zlabel!(plt, "z"); title!(plt, "AIF Robot Trajectory (3D)")
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

    # Optional: 3D Panda arm replay of the AIF 2D trajectory.
    if get(params, :renderarm, false)
        try
            render_panda_arm_from_history(res.history, params)
        catch err
            try
                println("Panda arm render failed: ", err)
                flush(stdout)
            catch
            end
        end
    end
end

main()
