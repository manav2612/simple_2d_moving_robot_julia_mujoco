# `src/utils/math.jl` — Math utilities

## Purpose

Provides mathematical helpers used across the Active Inference pipeline: normalization, Gaussian densities, entropy, softmax, and clamping.

## Exports

| Symbol | Description |
|--------|-------------|
| `normalize!` | Normalize a probability vector in place |
| `gaussian_pdf` | Univariate or multivariate (diagonal) Gaussian PDF |
| `gaussian_entropy` | Entropy of a Gaussian |
| `softmax` | Numerically stable softmax |
| `clamp_vec` | Clamp vector elements to bounds |

## Functions

### `normalize!(p)`

Ensures `p` sums to 1. If the sum is non-positive, replaces with uniform distribution.

### `gaussian_pdf(x, μ, σ²)`

Computes the Gaussian PDF.

### `gaussian_entropy(σ²)`

Computes entropy of a univariate Gaussian.

### `softmax(x)`

Numerically stable softmax with subtraction of max for stability.
