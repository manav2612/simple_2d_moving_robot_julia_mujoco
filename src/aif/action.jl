"""
Action representation and conversion for MuJoCo control.
"""
module Action

using LinearAlgebra

export to_control, clamp_action

"""Convert 2D velocity action to MuJoCo control signal [vx, vy]."""
function to_control(action::AbstractVector; scale::Real = 1.0, ctrl_lim::Real = 1.5)
    ctrl = action .* scale
    return clamp.(ctrl, -ctrl_lim, ctrl_lim)
end

"""Clamp action to limits."""
function clamp_action(action::AbstractVector; lim::Real = 1.5)
    return clamp.(action, -lim, lim)
end

end
