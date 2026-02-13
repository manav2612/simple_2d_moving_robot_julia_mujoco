"""
Configuration parameters for AIF MuJoCo robot experiments.
Per-dimension (x, y, z) noise supported for obs_noise and process_noise.
"""
module Configs

export default_config, config_goal_seeking, config_exploration

"""Default configuration (tuned for stable 3D goal-reaching)."""
function default_config()
    return (
        steps = 500,
        goal = [0.8, 0.8, 0.4],
        init_pos = [-0.5, -0.5, 0.2],
        obs_noise = 0.01,           # scalar or [σ²_x, σ²_y, σ²_z]
        process_noise = 0.005,     # lower = more stable belief
        γ = 1.2,                    # pragmatic weight (goal-seeking)
        β = 0.05,                   # epistemic weight (reduced for less chaos)
        ctrl_scale = 4.0,
        nsteps_per_ctrl = 5,
        seed = 42,
        verbose = true,
    )
end

"""Emphasize goal-seeking (higher γ)."""
function config_goal_seeking()
    c = default_config()
    return merge(c, (γ = 2.0, β = 0.05))
end

"""Emphasize exploration (higher β)."""
function config_exploration()
    c = default_config()
    return merge(c, (γ = 0.5, β = 0.3))
end

end
