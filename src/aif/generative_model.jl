"""
Generative model: transition and observation likelihood for Active Inference.
3D state (x, y, z) with per-dimension noise.
"""
module GenerativeModel

using LinearAlgebra

export predict_transition, observation_likelihood, predict_observation

"""Predict next state: s' = s + u (3D)."""
function predict_transition(s::AbstractVector, u::AbstractVector; process_noise = 0.01)
    return s[1:3] .+ u[1:3]
end

"""Observation likelihood P(o|s) - diagonal Gaussian, per-axis noise.
obs_noise: scalar or [σ²_x, σ²_y, σ²_z]
"""
function observation_likelihood(obs::AbstractVector, state::AbstractVector; obs_noise = 0.01)
    n = min(3, length(obs), length(state))
    σ² = obs_noise isa Real ? fill(obs_noise, n) : Float64.(obs_noise[1:n])
    d = obs[1:n] .- state[1:n]
    # Per-dimension: prod_i exp(-d_i²/(2σ²_i)) / sqrt(2πσ²_i)
    log_lik = -0.5 * sum(d.^2 ./ σ² .+ log.(2 * π .* σ²))
    return exp(log_lik)
end

"""Predict observation from state (noiseless mean)."""
function predict_observation(state::AbstractVector)
    return copy(state)
end

end
