"""
Math utilities for Active Inference and control.
"""
module MathUtils

using LinearAlgebra

export normalize!, gaussian_pdf, gaussian_entropy, softmax, clamp_vec

"""Normalize a probability vector in place."""
function normalize!(p::AbstractVector{<:Real})
    s = sum(p)
    if s <= 0
        fill!(p, 1.0 / length(p))
    else
        p ./= s
    end
    return p
end

"""Gaussian PDF: p(x | μ, σ²)."""
function gaussian_pdf(x::Real, μ::Real, σ²::Real)
    σ² <= 0 && return 0.0
    denom = sqrt(2 * π * σ²)
    z = (x - μ)^2 / (2 * σ²)
    return exp(-z) / denom
end

"""Multivariate Gaussian PDF (diagonal covariance)."""
function gaussian_pdf(x::AbstractVector, μ::AbstractVector, σ²::AbstractVector)
    n = length(x)
    prod = 1.0
    for i in 1:n
        prod *= gaussian_pdf(x[i], μ[i], σ²[i])
    end
    return prod
end

"""Entropy of univariate Gaussian: 0.5 * log(2πeσ²)."""
function gaussian_entropy(σ²::Real)
    σ² <= 0 && return 0.0
    return 0.5 * (log(2 * π * exp(1) * σ²))
end

"""Softmax with numerical stability."""
function softmax(x::AbstractVector{<:Real})
    m = maximum(x)
    exp_x = exp.(x .- m)
    return exp_x ./ sum(exp_x)
end

"""Clamp vector to bounds."""
function clamp_vec(v::AbstractVector, lo::Real, hi::Real)
    return clamp.(v, lo, hi)
end

end
