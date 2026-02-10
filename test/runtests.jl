using Test
using AIFMuJoCoRobot

@testset "Belief predict and update" begin
    # initialize belief
    prior_mean = [0.0, 0.0]
    prior_cov = [0.1, 0.2]
    b = AIFMuJoCoRobot.init_belief(prior_mean, prior_cov)

    # predict forward with an action
    action = [0.5, -0.2]
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


@testset "CLI render flag" begin
    proj = joinpath(@__DIR__, "..")
    # Unset DISPLAY/WAYLAND_DISPLAY to force headless branch and avoid hanging visualiser in CI
    cmd = `env -u DISPLAY -u WAYLAND_DISPLAY julia --project=$proj $proj/scripts/run_cli.jl --render --steps 2 --save_plot "" --verbose false`
    out = read(cmd, String)
    @test any(x->occursin(x, out), ["Render flag enabled", "No DISPLAY detected", "Launching MuJoCo visualizer"])
end
