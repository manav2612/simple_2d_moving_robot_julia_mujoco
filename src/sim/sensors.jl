"""
Sensor model: extract observations from MuJoCo state.
"""
module Sensors

using LinearAlgebra

export read_position, read_observation

"""Read 2D position from MuJoCo qpos (first two elements for slide joints)."""
function read_position(qpos::AbstractVector)
    return [qpos[1], qpos[2]]
end

"""Read full observation (position + optional noise).

`obs_noise` is interpreted as observation variance (σ²). When adding noise
we use standard deviation `sqrt(obs_noise)` so that other modules that treat
`obs_noise` as variance remain consistent with the Bayesian formulas.
"""
function read_observation(qpos; obs_noise::Real = 0.0, rng = nothing)
    qpos_vec = vec(collect(qpos))
    pos = read_position(qpos_vec)
    if obs_noise > 0 && rng !== nothing
        pos = pos .+ sqrt(obs_noise) .* randn(rng, 2)
    end
    return pos
end

end
