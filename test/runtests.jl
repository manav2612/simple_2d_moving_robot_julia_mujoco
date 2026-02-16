using Test
using AIFMuJoCoRobot

@testset "Belief predict and update" begin
    # initialize belief (3D)
    prior_mean = [0.0, 0.0, 0.0]
    prior_cov = [0.1, 0.2, 0.15]
    b = AIFMuJoCoRobot.init_belief(prior_mean, prior_cov)

    # predict forward with an action
    action = [0.5, -0.2, 0.1]
    process_noise = 0.05
    AIFMuJoCoRobot.predict_belief!(b, action; process_noise = process_noise)

    @test b.mean == prior_mean .+ action
    @test b.cov == prior_cov .+ process_noise

    # create an observation equal to the predicted true state
    obs = copy(b.mean)
    obs_noise = 0.01

    # compute expected posterior analytically (diagonal Gaussian)
    prior_prec = 1.0 ./ (prior_cov .+ process_noise)
    lik_prec = 1.0 / obs_noise
    post_prec = prior_prec .+ lik_prec
    expected_cov = 1.0 ./ post_prec
    expected_mean = (prior_prec .* (prior_mean .+ action) .+ lik_prec .* obs) ./ post_prec

    AIFMuJoCoRobot.update_belief!(b, obs; obs_noise = obs_noise)

    @test isapprox(b.cov, expected_cov; atol=1e-12, rtol=1e-8)
    @test isapprox(b.mean, expected_mean; atol=1e-12, rtol=1e-8)
end


@testset "RxInfer backend matches analytic (single step)" begin
    # One predict+update step: RxInfer streaming filter should match the
    # closed-form diagonal-Gaussian posterior.
    prior_mean = [0.0, 0.0, 0.0]
    prior_cov  = [0.1, 0.2, 0.15]
    obs_noise  = 0.01
    process_noise = 0.05
    ctrl   = [0.5, -0.2, 0.1]

    # ── analytic reference ──
    b = AIFMuJoCoRobot.init_belief(prior_mean, prior_cov)
    AIFMuJoCoRobot.predict_belief!(b, ctrl; process_noise = process_noise)
    predicted_mean = copy(b.mean)
    predicted_cov  = copy(b.cov)
    obs = copy(predicted_mean)                         # noiseless observation
    AIFMuJoCoRobot.update_belief!(b, obs; obs_noise = obs_noise)
    analytic_mean = copy(b.mean)
    analytic_cov  = copy(b.cov)

    # ── RxInfer filter ──
    rxf = AIFMuJoCoRobot.RxInferFilter.init_rxinfer_filter(
        prior_mean, prior_cov;
        obs_noise = obs_noise, process_noise = process_noise,
    )
    rx_mean, rx_var = AIFMuJoCoRobot.RxInferFilter.rxinfer_step!(rxf, ctrl, obs)
    AIFMuJoCoRobot.RxInferFilter.rxinfer_stop!(rxf)

    @test isapprox(rx_mean, analytic_mean; atol = 1e-6)
    @test isapprox(rx_var, analytic_cov;   atol = 1e-6)
end

@testset "RxInfer backend matches analytic (multi-step)" begin
    prior_mean = [1.0, -1.0, 0.5]
    prior_cov  = [0.05, 0.05, 0.05]
    obs_noise  = 0.02
    process_noise = 0.01

    # sequences of controls and observations (5 steps)
    ctrls = [[0.1, -0.1, 0.05], [0.2, 0.0, -0.1], [-0.05, 0.15, 0.0],
             [0.0, 0.0, 0.1], [0.1, 0.1, 0.1]]
    obss  = [[1.1, -1.1, 0.55], [1.25, -1.05, 0.5], [1.15, -0.9, 0.5],
             [1.15, -0.9, 0.6], [1.3, -0.8, 0.7]]

    # ── analytic ──
    b = AIFMuJoCoRobot.init_belief(prior_mean, prior_cov)
    for (c, o) in zip(ctrls, obss)
        AIFMuJoCoRobot.predict_belief!(b, c; process_noise = process_noise)
        AIFMuJoCoRobot.update_belief!(b, o; obs_noise = obs_noise)
    end
    analytic_mean = copy(b.mean)
    analytic_cov  = copy(b.cov)

    # ── RxInfer ──
    rxf = AIFMuJoCoRobot.RxInferFilter.init_rxinfer_filter(
        prior_mean, prior_cov;
        obs_noise = obs_noise, process_noise = process_noise,
    )
    local rx_mean, rx_var
    for (c, o) in zip(ctrls, obss)
        rx_mean, rx_var = AIFMuJoCoRobot.RxInferFilter.rxinfer_step!(rxf, c, o)
    end
    AIFMuJoCoRobot.RxInferFilter.rxinfer_stop!(rxf)

    @test isapprox(rx_mean, analytic_mean; atol = 1e-5)
    @test isapprox(rx_var, analytic_cov;   atol = 1e-5)
end

@testset "CLI render flag" begin
    proj = joinpath(@__DIR__, "..")
    # Unset DISPLAY/WAYLAND_DISPLAY to force headless branch and avoid hanging visualiser in CI
    cmd = `env -u DISPLAY -u WAYLAND_DISPLAY julia --project=$proj $proj/scripts/run_cli.jl --render --steps 2 --save_plot "" --verbose false`
    out = read(cmd, String)
    @test any(x->occursin(x, out), ["Render flag enabled", "No DISPLAY detected", "Launching MuJoCo visualizer"])
end
