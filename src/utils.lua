local M = {}
local robot_wrapper = require("src.wrapper.robot_wrapper")
local vector = require("src.vector")

function M.translational_to_differential(vel, ang)
	local L = robot.wheels.axis_length
	local left_v = vel - ang * L / 2
	local right_v = vel + ang * L / 2
	return left_v, right_v
end
return M
