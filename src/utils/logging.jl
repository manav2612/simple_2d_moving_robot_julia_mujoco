"""
Logging utilities for simulation.
"""
module LoggingUtils

using Printf

export log_step, log_summary

"""Log a single simulation step."""
function log_step(step::Int, pos::AbstractVector, goal::AbstractVector, belief_mean::AbstractVector, efe::Real)
    dist = sqrt(sum((pos .- goal) .^ 2))
    @printf("[Step %4d] pos=(%.3f, %.3f) goal=(%.3f, %.3f) dist=%.4f belief=(%.3f, %.3f) EFE=%.4f\n",
            step, pos[1], pos[2], goal[1], goal[2], dist,
            belief_mean[1], belief_mean[2], efe)
end

"""Log simulation summary."""
function log_summary(total_steps::Int, final_dist::Real, converged::Bool)
    @printf("\n=== Simulation Summary ===\n")
    @printf("Total steps: %d\n", total_steps)
    @printf("Final distance to goal: %.4f\n", final_dist)
    @printf("Converged: %s\n", converged ? "Yes" : "No")
end

end
