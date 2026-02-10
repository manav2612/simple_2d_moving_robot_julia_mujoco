"""
Generative model: transition and observation likelihood for Active Inference.
"""
module GenerativeModel

using LinearAlgebra

export predict_transition, observation_likelihood, predict_observation

"""Predict next state given current state and action: s' = s + u + noise."""
function predict_transition(s::AbstractVector, u::AbstractVector; process_noise::Real = 0.01)
    return s .+ u
end

"""Observation likelihood P(o|s) - Gaussian around state with observation noise."""
function observation_likelihood(obs::AbstractVector, state::AbstractVector; obs_noise::Real = 0.01)
    n = length(obs)
    d = obs .- state
    dist_sq = sum(d .^ 2)
    σ² = obs_noise
    return exp(-dist_sq / (2 * σ²)) / (sqrt(2 * π * σ²)^n)
end

"""Predict observation from state (noiseless mean)."""
function predict_observation(state::AbstractVector)
    return copy(state)
end

end
