local robot_wrapper = require("src.wrapper.robot_wrapper")
local logger = require("src.wrapper.logger")
local utils = require("src.utils")
local perceptual_schemas = require("src.perceptual_schemas")
local vector = require("src.vector")

local VMAX = 15
local MAX_VELOCITY = 15
local S = 1

local n_steps = 0

-- the limit_v function is used to limit the velocity of the robot
local function limit_v(v)
	if v > VMAX then
		return VMAX
	elseif v < -VMAX then
		return -VMAX
	else
		return v
	end
end

function init()
	n_steps = 0
end

-- the uniform_field function is used to calculate the uniform field
local function uniform_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle }
end

-- the attractive_field function is used to calculate the attractive field
local function attractive_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle }
end

-- the repulsive_field function is used to calculate the repulsive field
local function repulsive_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = -angle }
end

-- the tangential_field function is used to calculate the tangential field
local function tangential_field(strength, angle)
	local length = 0
	local angle_ret = 0
	-- this check is used to avoid making the robot turn in the opposite direction
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

-- the avoid_obstacles function is gets the value from the perceptual_schema and returns the polar value
local function avoid_obstacles()
	local closest_prox = perceptual_schemas.sum_proximity_sensors()
	return { strength = closest_prox.length, angle = closest_prox.angle }
end

-- the go_towards_light function is gets the value from the perceptual_schema and returns the polar value
local function go_towards_light()
	local lights = perceptual_schemas.sum_light_sensors()
	return { strength = lights.length, angle = lights.angle }
end

function step()
	n_steps = n_steps + 1
	local left_vel = 0
	local right_vel = 0
	-- the polar values are calculated for the obstacle and light
	local obstacle_polar_val = avoid_obstacles()
	local light_polar_val = go_towards_light()
	-- all the fields are calculated and summed up
	local obstacle_repulsion = repulsive_field(obstacle_polar_val.strength, obstacle_polar_val.angle)
	local light_attraction = attractive_field(light_polar_val.strength, light_polar_val.angle)
	local go_straight = uniform_field(math.max(0.5 - light_polar_val.strength, 0), 0)
	local obstacle_tangential = tangential_field(obstacle_polar_val.strength * 10, obstacle_polar_val.angle)
	local sum = vector.vec2_polar_sum(go_straight, obstacle_repulsion)
	sum = vector.vec2_polar_sum(sum, light_attraction)
	sum = vector.vec2_polar_sum(sum, obstacle_tangential)
	-- the differential velocities are calculated from the polar values
	left_vel, right_vel = utils.translational_to_differential(sum.length, sum.angle)
	-- the velocities are scaled to the max velocity
	local maxValue = math.max(left_vel, right_vel)
	local scaleFactor = MAX_VELOCITY / maxValue
	left_vel = left_vel * scaleFactor
	right_vel = right_vel * scaleFactor

	-- limiting the max velocity
	left_vel = limit_v(left_vel)
	right_vel = limit_v(right_vel)

	-- the velocities are applied to the robot
	robot_wrapper.wheels.set_velocity(left_vel, right_vel)
end

function reset()
	n_steps = 0
end

function destroy()
	x = robot.positioning.position.x
	y = robot.positioning.position.y
	d = math.sqrt((x - 2) ^ 2 + y ^ 2)
	print("f_distance " .. d)
end
