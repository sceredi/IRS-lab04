-- Put your global variables here
local robot_wrapper = require("src.wrapper.robot_wrapper")
local logger = require("src.wrapper.logger")
local utils = require("src.utils")
local perceptual_schemas = require("src.perceptual_schemas")
local vector = require("src.vector")

local MAX_VELOCITY = 15
local D = 1

local n_steps = 0

--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
	n_steps = 0
	logger.log("lets go")
end

local function uniform_field(length, angle)
	return { length = math.min(length, D) / D, angle = angle }
end

local function attraction_field(distance, angle)
	local length = 0
	if distance > 0 then
		length = math.min(distance, D) / D
	else
		length = 0
	end
	return { length = length, angle = angle }
end

local function repulsive_field(distance, angle)
	local length = 0
	if distance > 0 then
		length = math.min(distance, D) / D
	else
		length = 0
	end
	return { length = length, angle = -angle }
end

local function tangential_field(distance, angle)
	local length = 0
	local angle_ret = 0
	if angle > 0 then
		angle_ret = angle - math.pi / 2
	else
		angle_ret = angle + math.pi / 2
	end
	if distance > 0 then
		length = math.min(distance, D) / D
	else
		length = 0
	end
	return { length = length, angle = angle_ret }
end

local function avoid_obstacles()
	local closest_prox = perceptual_schemas.sum_proximity_sensors()
	return { distance = closest_prox.length, angle = closest_prox.angle }
end

local function go_towards_light()
	local lights = perceptual_schemas.two_most_bright_lights()
	local first = lights.first
	local second = lights.second
	local light = vector.vec2_polar_sum(first, second)
	return { distance = light.length, angle = light.angle }
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	n_steps = n_steps + 1
	local left_vel = 0
	local right_vel = 0
	local obstacle_polar_val = avoid_obstacles()
	local light_polar_val = go_towards_light()
	local obstacle_repulsion = repulsive_field(obstacle_polar_val.distance, obstacle_polar_val.angle)
	local light_attraction = attraction_field(light_polar_val.distance, light_polar_val.angle)
	local go_straight = uniform_field((1 - light_attraction.length) * D, 0)
	local obstacle_tangential = tangential_field(obstacle_polar_val.distance * 1.2, obstacle_polar_val.angle)
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
