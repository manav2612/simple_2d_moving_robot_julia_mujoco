# `src/aif/rxinfer_filter.jl` -- RxInfer streaming filter

## Purpose

Provides an alternative inference backend for the Active Inference belief updates using RxInfer.jl's reactive message-passing engine. Each spatial axis (x, y, z) runs an independent 1D linear-Gaussian state-space model, matching the diagonal-covariance assumption of the analytic `BeliefState`.

## Exports

| Symbol | Description |
|--------|-------------|
| `RxInferFilter3D` | Struct holding 3 per-axis RxInfer streaming engines |
| `init_rxinfer_filter` | Create and start a 3-axis filter |
| `rxinfer_step!` | Push one (ctrl, obs) pair and return posterior |
| `rxinfer_stop!` | Stop engines and clean up subscriptions |

## Factor graph (per axis)

```
x_prev ~ Normal(mean = m_prev, variance = v_prev)   [prior from previous posterior]
x      ~ Normal(mean = x_prev + u, variance = q)    [transition: state + control]
y      ~ Normal(mean = x, variance = r)              [observation likelihood]
```

- `m_prev`, `v_prev`: set automatically via `@autoupdates` from the posterior `q(x)` of the previous step.
- `u`: applied control for the current step.
- `q`: process noise variance.
- `r`: observation noise variance.

## Streaming design

Each axis uses a single `RecentSubject{NamedTuple{(:y,:u)}}` stream. Pushing a `(y, u)` tuple triggers exactly one inference cycle per step, avoiding the `combineLatest` double-fire issue that would occur with two separate streams.

## Usage

```julia
filter = init_rxinfer_filter([0.0, 0.0, 0.0], [0.01, 0.01, 0.01];
    obs_noise = 0.01, process_noise = 0.005)

post_mean, post_var = rxinfer_step!(filter, ctrl_vector, obs_vector)

rxinfer_stop!(filter)
```

## Equivalence to analytic update

The RxInfer filter produces posteriors identical to the analytic diagonal-Gaussian Kalman update (verified to machine-epsilon accuracy in tests). The two backends are interchangeable via the `inference_backend` keyword in `run_simulation`.
