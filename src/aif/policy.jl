"""
Policy selection: choose action that minimizes Expected Free Energy.
Functions are defined in parent module (AIFMuJoCoRobot) scope.
"""
module Policy

using LinearAlgebra

# Get EFE and Beliefs from parent - use parentmodule to resolve at runtime
const EFE_mod = Base.parentmodule(@__MODULE__).EFE
const Beliefs_mod = Base.parentmodule(@__MODULE__).Beliefs

export select_action, get_action_set

"""Discrete set of 2D velocity actions."""
function get_action_set(step_size::Real = 0.15)
    actions = Vector{Vector{Float64}}()
    for dx in (-step_size, 0.0, step_size)
        for dy in (-step_size, 0.0, step_size)
            push!(actions, [dx, dy])
        end
    end
    return actions
end

"""Select action that minimizes EFE."""
function select_action(belief::Beliefs_mod.BeliefState, goal::AbstractVector;
                       actions::Vector{<:AbstractVector} = get_action_set(),
                       γ::Real = 1.0, β::Real = 0.1)
    best_action = actions[1]
    best_efe = Inf
    for a in actions
        efe = EFE_mod.compute_efe(belief.mean, belief.cov, a, goal; γ = γ, β = β)
        if efe < best_efe
            best_efe = efe
            best_action = a
        end
    end
    return copy(best_action)
end

end
