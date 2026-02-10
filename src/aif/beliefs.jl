"""
Belief state representation for Active Inference.
"""
module Beliefs

using LinearAlgebra

export BeliefState, init_belief, update_belief!, entropy
export predict_belief!

"""Gaussian belief over 2D state (position)."""
struct BeliefState
    mean::Vector{Float64}
    cov::Vector{Float64}  # diagonal covariance [σ²_x, σ²_y]
end

"""Initialize belief with given mean and uncertainty."""
function init_belief(mean::AbstractVector, cov::AbstractVector = [0.01, 0.01])
    return BeliefState(copy(mean), copy(cov))
end

"""Update belief mean (position) after observation."""
function update_belief!(b::BeliefState, obs::AbstractVector; obs_noise::Real = 0.01)
    # Simple Bayesian update for diagonal Gaussian: posterior precision = prior_precision + likelihood_precision
    prior_prec = 1.0 ./ b.cov
    lik_prec = 1.0 / obs_noise
    post_prec = prior_prec .+ lik_prec
    b.cov .= 1.0 ./ post_prec
    b.mean .= (prior_prec .* b.mean .+ lik_prec .* obs) ./ post_prec
    return b
end

"""Predict belief forward given an action using a simple additive transition model.

This applies the generative model `s' = s + u` and inflates covariance by
`process_noise` (interpreted as variance) on each dimension.
"""
function predict_belief!(b::BeliefState, action::AbstractVector; process_noise::Real = 0.01)
    b.mean .= b.mean .+ action
    b.cov .= b.cov .+ process_noise
    return b
end

"""Entropy of diagonal Gaussian belief."""
function entropy(b::BeliefState)
    n = length(b.mean)
    return 0.5 * n * log(2 * π * exp(1)) + 0.5 * sum(log.(b.cov))
end

end
