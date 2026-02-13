"""
Policy selection: choose action that minimizes Expected Free Energy.
3D action set (x, y, z) with finer resolution for smoother goal-reaching.
"""
module Policy

using LinearAlgebra

const EFE_mod = Base.parentmodule(@__MODULE__).EFE
const Beliefs_mod = Base.parentmodule(@__MODULE__).Beliefs

export select_action, get_action_set

"""Discrete set of 3D velocity actions (5×5×5 grid for finer control).
step_size: 0.08 gives smoother trajectories than 0.15.
"""
function get_action_set(step_size::Real = 0.08)
    actions = Vector{Vector{Float64}}()
    for dx in (-step_size, -step_size/2, 0.0, step_size/2, step_size)
        for dy in (-step_size, -step_size/2, 0.0, step_size/2, step_size)
            for dz in (-step_size, -step_size/2, 0.0, step_size/2, step_size)
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
