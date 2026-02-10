"""
Configuration parameters for AIF MuJoCo robot experiments.
"""
module Configs

export default_config, config_goal_seeking, config_exploration

"""Default configuration."""
function default_config()
    return (
        steps = 500,
        goal = [0.8, 0.8],
        init_pos = [-0.5, -0.5],
        obs_noise = 0.01,
        γ = 1.0,           # pragmatic weight (goal distance)
        β = 0.1,           # epistemic weight (curiosity)
        ctrl_scale = 5.0,
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
