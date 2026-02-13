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

"""Compute control from current belief and goal. Supports per-axis weights for EFE."""
function compute_control(belief::Beliefs.BeliefState, goal::AbstractVector;
                         γ::Real = 1.0, β::Real = 0.1, ctrl_scale::Real = 1.0,
                         axis_weights = nothing, ctrl_lim::Real = 1.2)
    action = Policy.select_action(belief, goal; γ = γ, β = β, axis_weights = axis_weights,
                                  ctrl_scale = ctrl_scale, ctrl_lim = ctrl_lim)
    ctrl = Action.to_control(action; scale = ctrl_scale, ctrl_lim = ctrl_lim)
    efe_val = EFE.compute_efe(belief.mean, belief.cov, action, goal;
                              γ = γ, β = β, axis_weights = axis_weights,
                              ctrl_scale = ctrl_scale, ctrl_lim = ctrl_lim)
    return (ctrl = ctrl, action = action, efe = efe_val)
end

end
