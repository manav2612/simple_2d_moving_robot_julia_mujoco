"""
Belief state representation for Active Inference.
3D state (x, y, z) with per-dimension uncertainty modelling.
"""
module Beliefs

using LinearAlgebra

export BeliefState, init_belief, update_belief!, entropy
export predict_belief!

# Covariance bounds to prevent numerical instability and runaway uncertainty
const COV_MIN = 1e-5
const COV_MAX = 2.0

"""Ensure scalar or vector noise is a 3D vector."""
_to_vec3(x::Real) = [Float64(x), Float64(x), Float64(x)]
_to_vec3(v::AbstractVector) = length(v) >= 3 ? Float64.(v[1:3]) : Float64.(v)

"""Gaussian belief over 3D state (position)."""
struct BeliefState
    mean::Vector{Float64}
    cov::Vector{Float64}  # diagonal covariance [σ²_x, σ²_y, σ²_z]
end

"""Initialize belief with given mean and uncertainty."""
function init_belief(mean::AbstractVector, cov::AbstractVector = [0.01, 0.01, 0.01])
    return BeliefState(copy(mean), clamp.(Float64.(cov[1:3]), COV_MIN, COV_MAX))
end

"""Update belief mean (position) after observation.
Per-dimension observation noise: obs_noise can be scalar or [σ²_x, σ²_y, σ²_z].
Posterior precision = prior_precision + likelihood_precision for each axis.
"""
function update_belief!(b::BeliefState, obs::AbstractVector; obs_noise = 0.01)
    σ²_obs = _to_vec3(obs_noise)
    # Per-dimension Bayesian update for diagonal Gaussian
    prior_prec = 1.0 ./ clamp.(b.cov, COV_MIN, COV_MAX)
    lik_prec = 1.0 ./ σ²_obs
    post_prec = prior_prec .+ lik_prec
    b.cov .= clamp.(1.0 ./ post_prec, COV_MIN, COV_MAX)
    b.mean .= (prior_prec .* b.mean .+ lik_prec .* obs[1:3]) ./ post_prec
    return b
end

"""Predict belief forward given an action using transition model s' = s + u.

Per-dimension process noise: process_noise can be scalar or [σ²_x, σ²_y, σ²_z].
Covariance is bounded to prevent chaotic growth.
"""
function predict_belief!(b::BeliefState, action::AbstractVector; process_noise = 0.01)
    σ²_proc = _to_vec3(process_noise)
    b.mean .= b.mean .+ action[1:3]
    b.cov .= clamp.(b.cov .+ σ²_proc, COV_MIN, COV_MAX)
    return b
end

"""Entropy of diagonal Gaussian belief."""
function entropy(b::BeliefState)
    n = length(b.mean)
    return 0.5 * n * log(2 * π * exp(1)) + 0.5 * sum(log.(b.cov))
end

end
