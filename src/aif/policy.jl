"""
Policy selection: choose action that minimizes Expected Free Energy.
3D action set (x, y, z) with finer resolution for smoother goal-reaching.
"""
module Policy

using LinearAlgebra

const EFE_mod = Base.parentmodule(@__MODULE__).EFE
const Beliefs_mod = Base.parentmodule(@__MODULE__).Beliefs

export select_action, get_action_set

"""Discrete set of 3D velocity actions (7×7×7 grid for smooth control).
step_size: 0.04 gives smoother trajectories than 0.08 by reducing quantization.
Grid: [-3s, -2s, -s, 0, s, 2s, 3s] per axis → 343 actions total.
"""
function get_action_set(step_size::Real = 0.04)
    actions = Vector{Vector{Float64}}()
    rng = -3:3
    for ix in rng
        dx = ix * step_size
        for iy in rng
            dy = iy * step_size
            for iz in rng
                dz = iz * step_size
                push!(actions, [dx, dy, dz])
            end
        end
    end
    return actions
end

"""Select action that minimizes EFE. Supports per-axis weights and ctrl_scale for correct prediction."""
function select_action(belief::Beliefs_mod.BeliefState, goal::AbstractVector;
                       actions::Vector{<:AbstractVector} = get_action_set(),
                       γ::Real = 1.0, β::Real = 0.1, axis_weights = nothing,
                       ctrl_scale::Real = 1.0, ctrl_lim::Real = 1.2)
    best_action = actions[1]
    best_efe = Inf
    for a in actions
        efe = EFE_mod.compute_efe(belief.mean, belief.cov, a, goal;
                                  γ = γ, β = β, axis_weights = axis_weights,
                                  ctrl_scale = ctrl_scale, ctrl_lim = ctrl_lim)
        if efe < best_efe
            best_efe = efe
            best_action = a
        end
    end
    return copy(best_action)
end

end
