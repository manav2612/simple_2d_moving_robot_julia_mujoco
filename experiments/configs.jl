"""
Configuration parameters for AIF MuJoCo robot experiments.
Per-dimension (x, y, z) noise supported for obs_noise and process_noise.
"""
module Configs

export default_config, config_goal_seeking, config_exploration

"""Default configuration (tuned for smooth, stable 3D goal-reaching).

`inference_backend`: `:analytic` (default) or `:rxinfer` for RxInfer streaming filter.
`action_alpha`: EMA smoothing weight (0–1). Lower = smoother trajectory. 0.3 recommended.
"""
function default_config()
    return (
        steps = 500,
        goal = [0.8, 0.8, 0.4],
        init_pos = [-0.5, -0.5, 0.2],
        robot_pos = [0.6, 0.0, 0.4],
        obs_noise = 0.005,          # scalar or [σ²_x, σ²_y, σ²_z]  (lowered for less jitter)
        process_noise = 0.002,      # lower = more stable belief prediction
        γ = 1.5,                    # pragmatic weight (goal-seeking)
        β = 0.02,                   # epistemic weight (low for smooth, deterministic paths)
        ctrl_scale = 3.0,           # moderate scaling to avoid amplifying discrete jumps
        nsteps_per_ctrl = 8,        # more physics sub-steps per control for smoother motion
        action_alpha = 0.3,         # EMA smoothing: 0.3 = strong smoothing (70% prev retained)
        seed = 42,
        verbose = true,
        inference_backend = :analytic,
    )
end

"""Emphasize goal-seeking (higher γ, low β, moderate smoothing)."""
function config_goal_seeking()
    c = default_config()
    return merge(c, (γ = 2.5, β = 0.01, action_alpha = 0.4))
end

"""Emphasize exploration (higher β, less smoothing)."""
function config_exploration()
    c = default_config()
    return merge(c, (γ = 0.5, β = 0.2, action_alpha = 0.6))
end

end
