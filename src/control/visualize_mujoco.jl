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

MuJoCo.init_visualiser()

model_path = AIFMuJoCoRobot.default_model_path()
model = MuJoCo.load_model(model_path)
data = MuJoCo.init_data(model)
MuJoCo.reset!(model, data)

# AIF state
goal = [0.8, 0.8]
init_pos = [-0.5, -0.5]
Random.seed!(42)
rng = Random.default_rng()

# Set initial position
q = collect(data.qpos)
q[1] = init_pos[1]
q[2] = init_pos[2]
data.qpos[:] = q

belief = AIFMuJoCoRobot.init_belief(init_pos, [0.01, 0.01])
obs_noise = 0.01
ctrl_scale = 1.0
nsteps_per_ctrl = 5
step_cnt = 0

belief = AIFMuJoCoRobot.init_belief(init_pos, [0.01, 0.01])
obs_noise = 0.01
ctrl_scale = 1.0
nsteps_per_ctrl = 5
step_cnt = 0

function controller!(m, d)
    global step_cnt
    step_cnt += 1
    pos = [d.qpos[1], d.qpos[2]]
    obs = pos .+ (obs_noise > 0 ? obs_noise .* randn(rng, 2) : zeros(2))
    AIFMuJoCoRobot.update_belief!(belief, obs; obs_noise = obs_noise)

    # Check if goal is reached
    dist = sqrt(sum((pos .- goal) .^ 2))
    if dist < 0.05
        println("Goal reached! Distance: $dist. Exiting simulation.")
        exit(0)
    end

    result = AIFMuJoCoRobot.AIFController.compute_control(belief, goal; γ = 1.0, β = 0.1, ctrl_scale = ctrl_scale)
    ctrl = result.ctrl
    if step_cnt % 20 == 0
        println("Pos: $(round.(pos, digits=3)) | Goal: $goal | Ctrl: $(round.(ctrl, digits=3))")
        flush(stdout)
    end

    # Position actuators: target = pos + ctrl (displacement)
    
println("Launching MuJoCo visualizer... Close window to exit.")
MuJoCo.visualise!(model, data, controller = controller!)
