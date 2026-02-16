#!/usr/bin/env julia
# MuJoCo GUI viewer for AIF robot simulation.
# Run: julia --project=. scripts/visualize_mujoco.jl
# Requires: MuJoCo.install_visualiser() (run once if needed)

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.instantiate()

using MuJoCo
using AIFMuJoCoRobot
using Random
using Printf

MuJoCo.init_visualiser()

model_path = AIFMuJoCoRobot.default_model_path()
model = MuJoCo.load_model(model_path)
data = MuJoCo.init_data(model)
MuJoCo.reset!(model, data)

# -------------------------
# AIF state (3D)
# -------------------------
goal = [0.8, 0.8, 0.4]
init_pos = [-0.5, -0.5, 0.2]

Random.seed!(42)
rng = Random.default_rng()

# Set initial position (x, y, z)
q = collect(data.qpos)
q[1] = init_pos[1]
q[2] = init_pos[2]
q[3] = init_pos[3]
data.qpos[:] = q

belief = AIFMuJoCoRobot.init_belief(init_pos, [0.01, 0.01, 0.01])
obs_noise = 0.01
ctrl_scale = 5.0
nsteps_per_ctrl = 5

# -------------------------
# Logging setup
# -------------------------
log_file = open("trajectory_log.csv", "w")
println(log_file, "step,robot_x,robot_y,robot_z,belief_x,belief_y,belief_z,goal_x,goal_y,goal_z,ctrl_x,ctrl_y,ctrl_z")

step_counter = 0

function controller!(m, d)
    global step_counter

    pos = [d.qpos[1], d.qpos[2], d.qpos[3]]
    obs = pos .+ (obs_noise > 0 ? sqrt(obs_noise) .* randn(rng, 3) : zeros(3))

    AIFMuJoCoRobot.update_belief!(belief, obs; obs_noise = obs_noise)

    result = AIFMuJoCoRobot.AIFController.compute_control(
        belief,
        goal;
        γ = 1.0,
        β = 0.1,
        ctrl_scale = ctrl_scale
    )

    ctrl = result.ctrl

    # Position actuators: target = pos + ctrl
    target = pos .+ ctrl
    d.ctrl[1] = clamp(target[1], -10.0, 10.0)
    d.ctrl[2] = clamp(target[2], -10.0, 10.0)
    d.ctrl[3] = clamp(target[3], -10.0, 10.0)

    # -------------------------
    # Logging
    # -------------------------
    belief_mean = belief.mean

    @printf("Step %d | Robot: (%.3f, %.3f) | Belief: (%.3f, %.3f) | Goal: (%.3f, %.3f)\n",
        step_counter,
        pos[1], pos[2],
        belief_mean[1], belief_mean[2],
        goal[1], goal[2]
    )
    flush(stdout)

    println(log_file,
        "$(step_counter),$(pos[1]),$(pos[2]),$(belief_mean[1]),$(belief_mean[2]),$(goal[1]),$(goal[2]),$(ctrl[1]),$(ctrl[2])"
    )

    step_counter += 1

    return nothing
end

println("Launching MuJoCo visualizer... Close window to exit.")
MuJoCo.visualise!(model, data, controller = controller!)

close(log_file)
