-- Put your global variables here
local robot_wrapper = require("src.wrapper.robot_wrapper")
local logger = require("src.wrapper.logger")
local utils = require("src.utils")
local perceptual_schemas = require("src.perceptual_schemas")
local vector = require("src.vector")

local MAX_VELOCITY = 15
local S = 1

local n_steps = 0

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
	n_steps = 0
	logger.log("lets go")
end

local function uniform_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle }
end

local function attraction_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle }
end

local function repulsive_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = -angle }
end

local function tangential_field(strength, angle)
	local length = 0
	local angle_ret = 0
	if angle > 0 then
		angle_ret = angle - math.pi / 2
	else
		angle_ret = angle + math.pi / 2
	end
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle_ret }
end

local function avoid_obstacles()
	local closest_prox = perceptual_schemas.sum_proximity_sensors()
	return { strength = closest_prox.length, angle = closest_prox.angle }
end

local function go_towards_light()
	local lights = perceptual_schemas.sum_light_sensors()
	return { strength = lights.length, angle = lights.angle }
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	n_steps = n_steps + 1
	local left_vel = 0
	local right_vel = 0
	local obstacle_polar_val = avoid_obstacles()
	local light_polar_val = go_towards_light()
	local obstacle_repulsion = repulsive_field(obstacle_polar_val.strength, obstacle_polar_val.angle)
	local light_attraction = attraction_field(light_polar_val.strength, light_polar_val.angle)
	local go_straight = uniform_field(0.5 - light_polar_val.strength, 0)
	local obstacle_tangential = tangential_field(obstacle_polar_val.strength * 2, obstacle_polar_val.angle)
	local sum = vector.vec2_polar_sum(go_straight, obstacle_repulsion)
	sum = vector.vec2_polar_sum(sum, light_attraction)
	sum = vector.vec2_polar_sum(sum, obstacle_tangential)
	left_vel, right_vel = utils.translational_to_differential(sum.length, sum.angle)
	local maxValue = math.max(left_vel, right_vel)
	local scaleFactor = MAX_VELOCITY / maxValue
	left_vel = left_vel * scaleFactor
	right_vel = right_vel * scaleFactor
	robot_wrapper.wheels.set_velocity(left_vel, right_vel)
end

--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	-- robot.wheels.set_velocity(left_v, right_v)
	n_steps = 0
end

--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
	-- put your code here
end
