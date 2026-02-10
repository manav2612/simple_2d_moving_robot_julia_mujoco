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
ctrl_scale = 1.0 # Reduced scale for stable movement
nsteps_per_ctrl = 5
step_cnt = 0

function controller!(m, d)
    global step_cnt
    step_cnt += 1

    # 1. Simulation: Read current physical state
    pos = [d.qpos[1], d.qpos[2]]

    # 2. Math/AIF: Update belief state (Gaussian) with noisy observation
    obs = pos .+ (obs_noise > 0 ? obs_noise .* randn(rng, 2) : zeros(2))
    AIFMuJoCoRobot.update_belief!(belief, obs; obs_noise = obs_noise)

    # 3. Robustness: Check if goal is reached
    dist = sqrt(sum((pos .- goal) .^ 2))
    if dist < 0.05
        println("SUCCESS: Goal reached at $pos! Distance: $dist. Exiting.")
        exit(0)
    end

    # 4. Control: Compute action based on Belief (Math) and Goal
    result = AIFMuJoCoRobot.AIFController.compute_control(belief, goal; γ = 1.0, β = 0.1, ctrl_scale = ctrl_scale)
    ctrl = result.ctrl
    
    # 5. Logging: Prove that Math (Belief) and Simulation (Pos) are working
    if step_cnt % 20 == 0
        println("Step $step_cnt | Real Pos: $(round.(pos, digits=3)) | Belief μ: $(round.(belief.mean, digits=3)) | Ctrl: $(round.(ctrl, digits=3))")
        flush(stdout)
    end

    # Position actuators: target = pos + ctrl (displacement)
    target = pos .+ ctrl
    d.ctrl[1] = clamp(target[1], -2.0, 2.0)
    d.ctrl[2] = clamp(target[2], -2.0, 2.0)
    return nothing
end

println("Launching MuJoCo visualizer... Close window to exit.")
MuJoCo.visualise!(model, data, controller = controller!)
