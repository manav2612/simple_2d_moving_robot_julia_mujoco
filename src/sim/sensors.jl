"""
Sensor model: extract observations from MuJoCo state.
"""
module Sensors

using LinearAlgebra

export read_position, read_observation

"""Read 3D position from MuJoCo qpos (first three elements for slide joints x, y, z)."""
function read_position(qpos::AbstractVector)
    return [qpos[1], qpos[2], qpos[3]]
end

"""Read full observation (position + optional noise).

obs_noise: scalar or [σ²_x, σ²_y, σ²_z] - variance per axis.
"""
function read_observation(qpos; obs_noise = 0.0, rng = nothing)
    qpos_vec = vec(collect(qpos))
    pos = read_position(qpos_vec)
    if rng === nothing
        return pos
    end
    σ² = obs_noise isa Real ? fill(obs_noise, 3) : Float64.(obs_noise[1:3])
    for i in 1:3
        σ²[i] > 0 && (pos[i] += sqrt(σ²[i]) * randn(rng))
    end
    return pos
end

end
