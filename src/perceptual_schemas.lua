local M = {}
local robot_wrapper = require("src.wrapper.robot_wrapper")

function M.closest_proximity_sensor()
	local closest = robot_wrapper.get_proximity_sensor_readings()[1]
	for i = 2, 24 do
		if i <= 6 or i >= 19 then
			if robot_wrapper.get_proximity_sensor_readings()[i].value > closest.value then
				closest = robot_wrapper.get_proximity_sensor_readings()[i]
			end
		end
	end
	return closest
end

function M.brightest_light()
	local brightest = robot_wrapper.get_light_sensor_readings()[1]
	for i = 2, 24 do
		if robot_wrapper.get_light_sensor_readings()[i].value > brightest.value then
			brightest = robot_wrapper.get_light_sensor_readings()[i]
		end
	end
	return brightest
end
return M
