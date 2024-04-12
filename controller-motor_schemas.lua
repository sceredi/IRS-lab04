-- Put your global variables here
local robot_wrapper = require("src.wrapper.robot_wrapper")
local logger = require("src.wrapper.logger")
local perceptual_schemas = require("src.perceptual_schemas")
local utils = require("src.utils")
local vector = require("src.vector")

local MAX_VELOCITY = 15
local D = 1

local n_steps = 0
local left_v = 0
local right_v = 0

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
	left_v = MAX_VELOCITY
	right_v = 0
	robot_wrapper.wheels.set_velocity(left_v, right_v)
	n_steps = 0
	logger.log("lets go")
end

local function attraction_field(distance, angle)
	local length = 0
	if distance <= D then
		length = (D - distance) / D
	else
		length = 0
	end
	return { length = length, angle = angle }
end

local function repulsive_field(distance, angle)
	local length = 0
	if distance <= D then
		length = (D - distance) / D
	else
		length = 0
	end
	return { length = length, angle = -angle }
end

local function avoid_obstacles()
	local closest_prox = perceptual_schemas.closest_proximity_sensor()
	if closest_prox.value > 0.1 then
		robot_wrapper.leds.set_all_colors("red")
		return { distance = -closest_prox.value, angle = closest_prox.angle }
	else
		robot_wrapper.leds.set_all_colors("black")
		return { distance = 0, angle = 0 }
	end
end

local function go_towards_light()
	local brightest_light = perceptual_schemas.brightest_light()
	if brightest_light.value > 0.015 then
		robot_wrapper.leds.set_all_colors("yellow")
		return { distance = brightest_light.value + 0.5, angle = brightest_light.angle }
	else
		robot_wrapper.leds.set_all_colors("black")
		return { distance = 0, angle = 0 }
	end
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	n_steps = n_steps + 1
	left_v = robot_wrapper.random.uniform(0, MAX_VELOCITY)
	right_v = robot_wrapper.random.uniform(0, MAX_VELOCITY)
	local left_vel = left_v
	local right_vel = right_v
	local obstacle_polar_val = avoid_obstacles()
	local light_polar_val = go_towards_light()
	local obstacle_polar = repulsive_field(obstacle_polar_val.distance, obstacle_polar_val.angle)
	local light_polar = attraction_field(light_polar_val.distance, light_polar_val.angle)
	local sum = vector.vec2_polar_sum(obstacle_polar, light_polar)
	if sum.length > 0 then
		left_vel, right_vel = utils.translational_to_differential(sum.length, sum.angle)
	end
	local maxValue = math.max(left_vel, right_vel)
	local scaleFactor = MAX_VELOCITY / maxValue
	left_vel = left_vel * scaleFactor
	right_vel = right_vel * scaleFactor
	logger.log("left_vel: " .. left_vel .. "\nright_vel: " .. right_vel)
	logger.log("\n\n")
	robot_wrapper.wheels.set_velocity(left_vel, right_vel)
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
end

--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
	-- put your code here
end
