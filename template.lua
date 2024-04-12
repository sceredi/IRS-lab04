-- Put your global variables here
local robot_wrapper = require("src.wrapper.robot_wrapper")
local logger = require("src.wrapper.logger")

local MOVE_STEPS = 15
local MAX_VELOCITY = 10
local LIGHT_THRESHOLD = 1.5
local color = "yellow"

local n_steps = 0
local left_v = 0
local right_v = 0

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
	left_v = MAX_VELOCITY
	right_v = 0
	robot_wrapper.wheels.set_velocity(left_v, right_v)
	robot_wrapper.leds.set_all_colors(color)
	n_steps = 0
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	n_steps = n_steps + 1
	if n_steps % MOVE_STEPS == 0 then
		logger.Log("Changing color")
		if color == "yellow" then
			color = "black"
		else
			color = "yellow"
		end
		logger.Log("New color: " .. color)
		robot_wrapper.leds.set_all_colors(color)
	end
	-- logger.Log("Gonna move by " .. left_v .. " " .. right_v)
	light_controller()
	collision_avoidance()
	-- logger.Log("Moving by " .. left_v .. " " .. right_v)
	robot_wrapper.wheels.set_velocity(left_v, right_v)
end

function collision_avoidance()
	-- local closest = { pos = 1, value = robot.proximity[1].value }
	local closest = {
		pos = 1,
		value = robot_wrapper.get_proximity_sensor_readings()[1].value,
		robot_wrapper.get_proximity_sensor_readings()[1].value,
	}
	for i = 1, #robot_wrapper.get_proximity_sensor_readings() do
		-- logger.Log("proximity " .. i .. "->" .. robot.proximity[i].value)
		if robot_wrapper.get_proximity_sensor_readings()[i].value > closest.value then
			closest = {
				pos = i,
				value = robot_wrapper.get_proximity_sensor_readings()[i].value,
			}
		end
	end
	-- logger.Log("closest " .. closest.pos .. " " .. closest.value)
	if closest.value >= 0.5 then
		if closest.pos <= 7 then
			left_v = 2
			right_v = -2
		elseif closest.pos >= 18 then
			left_v = -2
			right_v = 2
		end
	end
end

function light_controller()
	local biggest_light = { pos = 1, value = robot_wrapper.get_light_sensor_readings()[1].value }
	-- Looking if the light is in front
	for i = 2, #robot_wrapper.get_light_sensor_readings() do
		if robot_wrapper.get_light_sensor_readings()[i].value > biggest_light.value then
			biggest_light = {
				pos = i,
				value = robot_wrapper.get_light_sensor_readings()[i].value,
			}
		end
	end
	if biggest_light.pos <= 12 then
		left_v = 5
		right_v = 10
		-- robot.wheels.set_velocity(left_v, right_v)
	else
		left_v = 10
		right_v = 5
		-- robot.wheels.set_velocity(left_v, right_v)
	end
end

--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	left_v = MAX_VELOCITY
	right_v = 0
	-- robot.wheels.set_velocity(left_v, right_v)
	robot_wrapper.wheels.set_velocity(left_v, right_v)
	n_steps = 0
	robot_wrapper.leds.set_all_colors(color)
end

--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
	-- put your code here
end
