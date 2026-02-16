"""
Expected Free Energy (EFE) for Active Inference policy selection.
EFE = pragmatic (goal-seeking) - epistemic (exploration)
All terms use explicit 3D (x, y, z) per-dimension formulation.
"""
module EFE

using LinearAlgebra

export compute_efe, pragmatic_term, epistemic_term

"""Pragmatic term: expected squared distance to goal, per-axis (x, y, z).
predicted_pos = belief_mean + displacement (use ctrl_scale so displacement matches actual control).
"""
function pragmatic_term(belief_mean::AbstractVector, action::AbstractVector, goal::AbstractVector;
                        γ::Real = 1.0, axis_weights = nothing, ctrl_scale::Real = 1.0, ctrl_lim::Real = 1.2)
    displacement = clamp.(action[1:3] .* ctrl_scale, -ctrl_lim, ctrl_lim)
    predicted_pos = belief_mean[1:3] .+ displacement
    goal_3 = goal[1:3]
    err_sq = (predicted_pos .- goal_3) .^ 2
    w = axis_weights === nothing ? ones(3) : Float64.(axis_weights[1:3])
    return γ * sum(w .* err_sq)
end

"""Epistemic term: entropy-related, encourages uncertainty reduction.
Uses clamped covariance for numerical stability.
"""
function epistemic_term(belief_cov::AbstractVector; β::Real = 0.1)
    cov_safe = clamp.(belief_cov[1:3], 1e-8, 1e2)
    return -β * sum(log.(cov_safe))
end

"""Compute Expected Free Energy for a policy (action).
Minimize EFE → get close to goal (low pragmatic) and reduce uncertainty (high epistemic).
Uses ctrl_scale so predicted position matches actual applied control.
"""
function compute_efe(belief_mean::AbstractVector, belief_cov::AbstractVector,
                     action::AbstractVector, goal::AbstractVector;
                     γ::Real = 1.0, β::Real = 0.1, axis_weights = nothing,
                     ctrl_scale::Real = 1.0, ctrl_lim::Real = 1.2)
    pragmatic = pragmatic_term(belief_mean, action, goal; γ = γ, axis_weights = axis_weights,
                               ctrl_scale = ctrl_scale, ctrl_lim = ctrl_lim)
    epistemic = epistemic_term(belief_cov; β = β)
    return pragmatic - epistemic
end

end
