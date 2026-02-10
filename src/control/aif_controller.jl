"""
Active Inference controller: integrates belief, policy, and action.
"""
module AIFController

using LinearAlgebra
using ..Beliefs
using ..Policy
using ..Action
using ..EFE

export compute_control

"""Compute control from current belief and goal."""
function compute_control(belief::Beliefs.BeliefState, goal::AbstractVector;
                         γ::Real = 1.0, β::Real = 0.1, ctrl_scale::Real = 1.0)
    action = Policy.select_action(belief, goal; γ = γ, β = β)
    ctrl = Action.to_control(action; scale = ctrl_scale)
    efe_val = EFE.compute_efe(belief.mean, belief.cov, action, goal; γ = γ, β = β)
    return (ctrl = ctrl, action = action, efe = efe_val)
end

end
