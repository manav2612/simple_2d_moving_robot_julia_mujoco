"""
Expected Free Energy (EFE) for Active Inference policy selection.
EFE = E[ -log P(o|C) ] + E[ D_KL[Q(s'|u) || Q(s)] ] ≈ pragmatic + epistemic
"""
module EFE

using LinearAlgebra

export compute_efe, pragmatic_term, epistemic_term

"""Pragmatic term: expected squared distance to goal (negative reward)."""
function pragmatic_term(belief_mean::AbstractVector, action::AbstractVector, goal::AbstractVector;
                        γ::Real = 1.0)
    predicted_pos = belief_mean .+ action
    dist_sq = sum((predicted_pos .- goal) .^ 2)
    return γ * dist_sq
end

"""Epistemic term: expected reduction in uncertainty (simplified as entropy)."""
function epistemic_term(belief_cov::AbstractVector; β::Real = 0.1)
    # Higher covariance → higher epistemic value (seeking information)
    return -β * sum(log.(belief_cov .+ 1e-10))
end

"""Compute Expected Free Energy for a policy (action)."""
function compute_efe(belief_mean::AbstractVector, belief_cov::AbstractVector,
                     action::AbstractVector, goal::AbstractVector;
                     γ::Real = 1.0, β::Real = 0.1)
    pragmatic = pragmatic_term(belief_mean, action, goal; γ = γ)
    epistemic = epistemic_term(belief_cov; β = β)
    return pragmatic - epistemic  # minimize EFE: low pragmatic (close to goal), high epistemic (explore)
end

end
