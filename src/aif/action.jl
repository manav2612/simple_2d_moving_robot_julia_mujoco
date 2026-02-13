"""
Action representation and conversion for MuJoCo control.
"""
module Action

using LinearAlgebra

export to_control, clamp_action

"""Convert 3D velocity action to MuJoCo control signal [vx, vy, vz].
ctrl_lim prevents overshooting; 1.2 works well with finer action sets.
"""
function to_control(action::AbstractVector; scale::Real = 1.0, ctrl_lim::Real = 1.2)
    ctrl = action[1:3] .* scale
    return clamp.(ctrl, -ctrl_lim, ctrl_lim)
end

"""Clamp action to limits."""
function clamp_action(action::AbstractVector; lim::Real = 1.5)
    return clamp.(action, -lim, lim)
end

end
