"""
RxInfer-based online Bayesian filter for Active Inference belief updates.

Provides a 3-axis (x, y, z) streaming filter using RxInfer's reactive
message-passing engine.  Each axis runs an independent 1D linear-Gaussian
state-space model so the diagonal-covariance assumption matches the existing
analytic `BeliefState`.

Public API
----------
- `RxInferFilter3D`        – mutable struct holding 3 per-axis engines
- `init_rxinfer_filter`    – constructor
- `rxinfer_step!`          – predict+update for one timestep
- `rxinfer_stop!`          – clean shutdown of engines
"""
module RxInferFilter

using RxInfer
using Rocket
using Distributions

export RxInferFilter3D, init_rxinfer_filter, rxinfer_step!, rxinfer_stop!

# ─────────────────────────────────────────────────────────────
# 1.  Per-axis RxInfer model
# ─────────────────────────────────────────────────────────────
#
# State-space model (1-D, one timestep):
#   x_prev ~ Normal(mean = m_prev, variance = v_prev)   [prior from previous posterior]
#   x      ~ Normal(mean = x_prev + u, variance = q)    [transition: state + control]
#   y      ~ Normal(mean = x, variance = r)              [observation likelihood]
#
# `m_prev`, `v_prev`, `u`, `q`, `r` are provided as data / model arguments
# and updated each timestep via @autoupdates.

@model function axis_ssm(y, m_prev, v_prev, u, q, r)
    x_prev ~ Normal(mean = m_prev, variance = v_prev)
    x      ~ Normal(mean = x_prev + u, variance = q)
    y      ~ Normal(mean = x, variance = r)
end

# ─────────────────────────────────────────────────────────────
# 2.  Per-axis engine wrapper
# ─────────────────────────────────────────────────────────────

const YU = NamedTuple{(:y, :u), Tuple{Float64, Float64}}

mutable struct AxisFilter
    # Single atomic stream for (y, u) pairs – avoids combineLatest double-fire
    data_stream::Any       # RecentSubject{YU}
    # The RxInfer engine
    engine::Any            # RxInferenceEngine
    # Current posterior parameters (kept in sync via subscription)
    post_mean::Float64
    post_var::Float64
    # Subscription handle so we can unsubscribe on stop
    subscription::Any
end

"""Build one per-axis streaming RxInfer engine."""
function _make_axis_filter(;
    init_mean::Float64,
    init_var::Float64,
    obs_noise::Float64,
    process_noise::Float64,
)
    # Single stream emitting NamedTuple{(:y,:u)} so each push triggers
    # exactly one inference cycle (no combineLatest double-fire).
    data_stream = RecentSubject(YU)

    autoupdates = @autoupdates begin
        m_prev, v_prev = mean_var(q(x))
    end

    engine = infer(
        model          = axis_ssm(q = process_noise, r = obs_noise),
        datastream     = data_stream,
        autoupdates    = autoupdates,
        returnvars     = (:x,),
        keephistory    = 0,
        initialization = @initialization(q(x) = NormalMeanVariance(init_mean, init_var)),
        autostart      = false,
    )

    # Subscribe to posterior updates so we always have the latest values
    af = AxisFilter(data_stream, engine, init_mean, init_var, nothing)
    sub = subscribe!(engine.posteriors[:x], (posterior) -> begin
        af.post_mean = mean(posterior)
        af.post_var  = var(posterior)
    end)
    af.subscription = sub

    RxInfer.start(engine)
    return af
end

# ─────────────────────────────────────────────────────────────
# 3.  Three-axis wrapper
# ─────────────────────────────────────────────────────────────

mutable struct RxInferFilter3D
    axes::Vector{AxisFilter}       # length-3 (x, y, z)
    obs_noise::Vector{Float64}     # [r_x, r_y, r_z]
    process_noise::Vector{Float64} # [q_x, q_y, q_z]
end

"""
    init_rxinfer_filter(init_mean, init_cov; obs_noise, process_noise)

Create a 3-axis streaming RxInfer filter.

- `init_mean`: `[μ_x, μ_y, μ_z]`
- `init_cov`:  `[σ²_x, σ²_y, σ²_z]`  (diagonal variances)
- `obs_noise`: scalar or `[r_x, r_y, r_z]`
- `process_noise`: scalar or `[q_x, q_y, q_z]`
"""
function init_rxinfer_filter(
    init_mean::AbstractVector,
    init_cov::AbstractVector;
    obs_noise = 0.01,
    process_noise = 0.005,
)
    on = obs_noise isa Real ? fill(Float64(obs_noise), 3) : Float64.(obs_noise[1:3])
    pn = process_noise isa Real ? fill(Float64(process_noise), 3) : Float64.(process_noise[1:3])

    axes = AxisFilter[]
    for i in 1:3
        af = _make_axis_filter(;
            init_mean    = Float64(init_mean[i]),
            init_var     = Float64(init_cov[i]),
            obs_noise    = on[i],
            process_noise = pn[i],
        )
        push!(axes, af)
    end
    return RxInferFilter3D(axes, on, pn)
end

"""
    rxinfer_step!(filter, ctrl, obs) -> (post_mean, post_var)

Push one control + observation into the filter and return the updated
posterior `([μ_x,μ_y,μ_z], [σ²_x,σ²_y,σ²_z])`.

The call is synchronous: by the time it returns the reactive engine has
already propagated the messages and updated `post_mean`/`post_var` on
each axis.
"""
function rxinfer_step!(filter::RxInferFilter3D, ctrl::AbstractVector, obs::AbstractVector)
    for i in 1:3
        af = filter.axes[i]
        # Push (y, u) as a single atomic NamedTuple so exactly one
        # inference cycle fires per step.
        next!(af.data_stream, YU((Float64(obs[i]), Float64(ctrl[i]))))
    end
    post_mean = [filter.axes[i].post_mean for i in 1:3]
    post_var  = [filter.axes[i].post_var  for i in 1:3]
    return (post_mean, post_var)
end

"""Stop all three RxInfer engines and unsubscribe."""
function rxinfer_stop!(filter::RxInferFilter3D)
    for af in filter.axes
        try
            RxInfer.stop(af.engine)
        catch; end
        try
            unsubscribe!(af.subscription)
        catch; end
    end
end

end # module RxInferFilter
