local M = {}
local robot_wrapper = require("src.wrapper.robot_wrapper")
local vector = require("src.vector")

function M.translational_to_differential(vel, ang)
	local L = robot.wheels.axis_length
	local left_v = vel - ang * L / 2
	local right_v = vel + ang * L / 2
	return left_v, right_v
end

function M.closest_proximity_sensor()
	local closest = robot_wrapper.get_proximity_sensor_readings()[1]
	for i = 2, 24 do
		if i <= 6 or i >= 19 then
			if robot_wrapper.get_proximity_sensor_readings()[i].value > closest.value then
				closest = robot_wrapper.get_proximity_sensor_readings()[i]
			end
		end
	end
	return { length = closest.value, angle = closest.angle }
end

function M.brightest_light()
	local brightest = robot_wrapper.get_light_sensor_readings()[1]
	for i = 2, 24 do
		if robot_wrapper.get_light_sensor_readings()[i].value > brightest.value then
			brightest = robot_wrapper.get_light_sensor_readings()[i]
		end
	end
	return { length = brightest.value, angle = brightest.angle }
end

function M.two_most_bright_lights()
	local first = robot_wrapper.get_light_sensor_readings()[1]
	local second = robot_wrapper.get_light_sensor_readings()[2]
	if first.value < second.value then
		first, second = second, first
	end
	for i = 3, 24 do
		if robot_wrapper.get_light_sensor_readings()[i].value > first.value then
			second = first
			first = robot_wrapper.get_light_sensor_readings()[i]
		elseif robot_wrapper.get_light_sensor_readings()[i].value > second.value then
			second = robot_wrapper.get_light_sensor_readings()[i]
		end
	end
	return {
		first = { length = first.value, angle = first.angle },
		second = { length = second.value, angle = second.angle },
	}
end

function M.lights_sum()
	local first = robot_wrapper.get_light_sensor_readings()[1]
	local second = robot_wrapper.get_light_sensor_readings()[2]
	local sum = vector.vec2_polar_sum(
		{ length = first.value, angle = first.angle },
		{ length = second.value, angle = second.angle }
	)
	for i = 3, 24 do
		sum = vector.vec2_polar_sum(sum, {
			length = robot_wrapper.get_light_sensor_readings()[i].value,
			angle = robot_wrapper.get_light_sensor_readings()[i].angle,
		})
	end
	return sum
end
return M
