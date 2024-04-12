local M = {}

--- Base ground sensor readings.
-- The base ground sensor reads the color of the floor. It is a list of 8 readings,
-- each containing a table composed of value and offset. The value is either 0 or 1,
-- where 0 means black (or dark gray), and 1 means white (or light gray). The offset
-- corresponds to the position read on the ground by the sensor. The position is expressed
-- as a 2D vector stemming from the center of the robot. The vector coordinates are in cm.
-- The difference between this sensor and the robot.motor_ground is that this sensor returns
-- binary readings (0 = black/1 = white), while robot.motor_ground can distinguish different
-- shades of gray.
-- @field value number The value read by the sensor (0 for black, 1 for white).
-- @field offset table The position read on the ground by the sensor, expressed as a 2D vector.
-- @field offset.x number The x-coordinate of the position vector (in cm).
-- @field offset.y number The y-coordinate of the position vector (in cm).
---@class BaseGroundReading
---@field value number
---@field offset {x: number, y: number}

--- List of base ground sensor readings.
---@return BaseGroundReading[]
function M.get_base_ground_readings()
	return robot.base_ground
end

--- Distance scanner sensor readings.
-- The distance scanner is a rotating device with four sensors. Two sensors are short-range (4cm to 30cm)
-- and two are long-range (20cm to 150cm). Each sensor returns up to 6 values every time step, for a total
-- of 24 readings (12 short-range and 12 long-range). Each reading is a table composed of angle in radians
-- and distance in cm. The distance value can also be -1 or -2. When it is -1, it means that the object detected
-- by the sensor is closer than the minimum sensor range (4cm for short-range, 20cm for long-range).
-- When a sensor returns -2, it's because no object was detected at all.
--
-- @field angle number The angle at which the object is detected (in radians).
-- @field distance number The distance to the detected object (in cm).
--
---@class DistanceScannerReading
---@field angle number
---@field distance number

--- List of distance scanner sensor readings.
---@type DistanceScannerReading[]

--- List of distance scanner sensor readings.
---@return DistanceScannerReading[]
function M.get_distance_scanner_readings()
	return robot.distance_scanner
end

M.distance_scanner = {}

--- Enable the distance scanner.
function M.distance_scanner.enable()
	robot.distance_scanner.enable()
end

--- Disable the distance scanner.
function M.distance_scanner.disable()
	robot.distance_scanner.disable()
end

--- Set the angle of the distance scanner.
---@param angle number The angle to set (in radians).
function M.set_distance_scanner_angle(angle)
	robot.distance_scanner.set_angle(angle)
end

--- Set the RPM of the distance scanner.
---@param rpm number The RPM to set.
function M.set_distance_scanner_rpm(rpm)
	robot.distance_scanner.set_rpm(rpm)
end

--- Robot gripper control.
-- The gripper allows a robot to connect to objects such as boxes and cylinders, or other robots.
-- A robot attached to a passive object can transport it if it is light enough.
--
---@class GripperControl

M.gripper = {}
--- Locks the gripper positively.
function M.gripper.lock_positive()
	robot.gripper.lock_positive()
end

--- Locks the gripper negatively.
function M.gripper.lock_negative()
	robot.gripper.lock_negative()
end

--- Unlocks the gripper.
function M.gripper.unlock()
	robot.gripper.unlock()
end

--- String containing the id of the robot.
---@type string
M.id = robot.id

--- Sets the color of the robot LEDs.
M.leds = {}

--- Sets the color of a single LED.
---@param idx number The index of the LED to set (1-12 for the body LEDs, 13 for the beacon).
---@param color string|{number, number, number} The color to set. It can be expressed as a string ("red", "green", "blue", etc.) or as a triplet of numbers (r,g,b).
function M.leds.set_single_color(idx, color)
	robot.leds.set_single_color(idx, color)
end

--- Sets the color of all LEDs at once.
---@param color string|{number, number, number} The color to set for all LEDs. It can be expressed as a string ("red", "green", "blue", etc.) or as a triplet of numbers (r,g,b).
function M.leds.set_all_colors(color)
	robot.leds.set_all_colors(color)
end

--- Light sensor readings.
-- The light sensor allows the robot to detect light sources. The robot has 24 light sensors, equally distributed
-- in a ring around its body. Each sensor reading is composed of an angle in radians and a value in the range [0,1].
-- The angle corresponds to where the sensor is located in the body with respect to the front of the robot, which
-- is the local x axis. Regarding the value, 0 corresponds to no light being detected by a sensor, while values > 0
-- mean that light has been detected. The value increases as the robot gets closer to a light source.
--
-- @field angle number The angle at which the light source is detected (in radians).
-- @field value number The value representing the intensity of light detected (in the range [0,1]).
--
---@class LightSensorReading
---@field angle number
---@field value number

--- List of light sensor readings.
---@type LightSensorReading[]

--- List of light sensor readings.
---@return LightSensorReading[]
function M.get_light_sensor_readings()
	return robot.light
end

--- Motor ground sensor readings.
-- The motor ground sensor reads the color of the floor. It is a list of 4 readings, each containing a table composed of value and offset.
-- The value goes from 0 or 1, where 0 means black, and 1 means white. The offset corresponds to the position read on the ground by the sensor.
-- The position is expressed as a 2D vector stemming from the center of the robot. The vector coordinates are in cm.
-- The difference between this sensor and the robot.base_ground is that this sensor can distinguish different shades of gray, while robot.base_ground returns binary readings (0 = black/1 = white).
--
---@class MotorGroundSensorReading
---@field value number The value read by the sensor (0 for black, 1 for white).
---@field offset {x: number, y: number} The position read on the ground by the sensor, expressed as a 2D vector.

--- List of motor ground sensor readings.
---@type MotorGroundSensorReading[]

--- List of motor ground sensor readings.
---@return MotorGroundSensorReading[]
function M.get_motor_ground()
	return robot.motor_ground
end

--- Proximity sensor readings.
-- The proximity sensors detect objects around the robots. The sensors are 24 and are equally distributed
-- in a ring around the robot body. Each sensor has a range of 10cm and returns a reading composed of an angle
-- in radians and a value in the range [0,1]. The angle corresponds to where the sensor is located in the body
-- with respect to the front of the robot, which is the local x axis. Regarding the value, 0 corresponds to no
-- object being detected by a sensor, while values > 0 mean that an object has been detected. The value increases
-- as the robot gets closer to the object.
--
-- @field angle number The angle at which an object is detected (in radians).
-- @field value number The value representing the proximity of the detected object (in the range [0,1]).
--
---@class ProximitySensorReading
---@field angle number
---@field value number

--- List of proximity sensor readings.
---@type ProximitySensorReading[]

--- List of proximity sensor readings.
---@return ProximitySensorReading[]
function M.get_proximity_sensor_readings()
	return robot.proximity
end

--- Random number generation.
M.random = {}

--- Generates a random number from a Bernoulli distribution.
---@param p number The probability of success (default: 0.5).
---@return number The generated random number (0 or 1).
function M.random.bernoulli(p)
	return robot.random.bernoulli(p)
end

--- Generates a random number from an exponential distribution.
---@param m number The mean of the distribution.
---@return number The generated random number.
function M.random.exponential(m)
	return robot.random.exponential(m)
end

--- Generates a random number from a Gaussian distribution.
---@param s number The standard deviation of the distribution.
---@param m number The mean of the distribution (default: 0).
---@return number The generated random number.
function M.random.gaussian(s, m)
	return robot.random.gaussian(s, m)
end

--- Generates a random number from a uniform distribution.
---@param min number The minimum value of the range (default: 0).
---@param max number The maximum value of the range (default: 1).
---@return number The generated random number.
function M.random.uniform(min, max)
	if max < min then
		min, max = max, min
	end
	return robot.random.uniform(min, max)
end

--- Generates a random integer number from a uniform distribution.
---@param min integer The minimum value of the range.
---@param max integer The maximum value of the range.
---@return integer The generated random integer number.
function M.random.uniform_int(min, max)
	if max < min then
		min, max = max, min
	end
	return robot.random.uniform_int(min, max)
end

--- Range-and-bearing communication.
M.range_and_bearing = {}

--- Sets the data to broadcast.
---@param idx? number|table If provided as a number, sets the idx-th byte to the value of data. If provided as a table, data must contain exactly 10 numbers in the range [0,255].
---@param data? number|{number,...} If idx is provided as a number, sets the idx-th byte to this value. If idx is provided as a table, data must contain exactly 10 numbers in the range [0,255].
function M.range_and_bearing.set_data(idx, data)
	robot.range_and_bearing.set_data(idx, data)
end

--- Receives messages from nearby robots.
---@return table[] A list of messages received from nearby robots. Each message is a table composed of: data (the 10-byte message payload), horizontal_bearing (the angle between the robot's local x axis and the position of the message source, in radians), vertical_bearing (the angle between the message source and the robot's xy plane, in radians), and range (the distance of the message source in cm).
function M.range_and_bearing.receive_messages()
	-- TODO: not yet implemented
end

--- Robot wheel motion control.
-- The real robot moves using two sets of wheels and tracks called treels. For simplicity, we treat the treels like normal wheels.
-- To move the robot, use set_velocity(l,r) where l and r are the left and right wheel velocity, respectively. By 'wheel velocity'
-- we mean linear velocity. In other words, if you say set_velocity(5,5), the robot will move forward at 5cm/s.
-- You can get some information about robot motion and wheels, too. axis_length is the distance between the two wheels in cm.
-- velocity_left and velocity_right store the current wheel velocity. distance_left and distance_right store the linear distance
-- covered by the wheels in the last time step.
--
-- @field axis_length number The distance between the two wheels (in cm).
-- @field velocity_left number The current velocity of the left wheel (in cm/s).
-- @field velocity_right number The current velocity of the right wheel (in cm/s).
-- @field distance_left number The linear distance covered by the left wheel in the last time step (in cm).
-- @field distance_right number The linear distance covered by the right wheel in the last time step (in cm).
--
---@class WheelMotionControl
---@field axis_length number
---@field velocity_left number
---@field velocity_right number
---@field distance_left number
---@field distance_right number

M.wheels = {}

--- Set the velocity of the robot wheels.
---@param l number The velocity of the left wheel (in cm/s).
---@param r number The velocity of the right wheel (in cm/s).
function M.wheels.set_velocity(l, r)
	robot.wheels.set_velocity(l, r)
end

--- Get information about robot motion and wheels.
---@return WheelMotionControl
function M.wheels.get_wheel_info()
	return {
		axis_length = robot.wheels.axis_length,
		velocity_left = robot.wheels.velocity_left,
		velocity_right = robot.wheels.velocity_right,
		distance_left = robot.wheels.distance_left,
		distance_right = robot.wheels.distance_right,
	}
end

--- Turret control.
M.turret = {}

--- Sets the gripper to position control mode and rotates it to the specified angle.
---@param angle number The angle to rotate the gripper to (in radians).
function M.turret.set_position_control_mode(angle)
	robot.turret.set_position(angle)
end

--- Sets the gripper to speed control mode and sets its rotational speed.
---@param speed number The rotational speed of the gripper (in radians per second).
function M.turret.set_speed_control_mode(speed)
	robot.turret.set_speed(speed)
end

--- Rotates the gripper to the specified angle.
---@param angle number The angle to rotate the gripper to (in radians).
function M.turret.set_rotation(angle)
	robot.turret.set_rotation(angle)
end

--- Sets the rotational speed of the gripper.
---@param speed number The rotational speed of the gripper (in radians per second).
function M.turret.set_rotation_speed(speed)
	robot.turret.set_rotation_speed(speed)
end

--- Sets the gripper to passive mode.
function M.turret.set_passive_mode()
	robot.turret.set_passive()
end

--- Colored blob omnidirectional camera.
M.colored_blob_omnidirectional_camera = {}

--- Enables the colored blob omnidirectional camera to start collecting data.
function M.colored_blob_omnidirectional_camera.enable()
	robot.colored_blob_omnidirectional_camera.enable()
end

--- Disables the colored blob omnidirectional camera to stop collecting data.
function M.colored_blob_omnidirectional_camera.disable()
	robot.colored_blob_omnidirectional_camera.disable()
end

--- Gets the list of colored blobs detected by the camera.
---@return table[] A list of colored blobs detected by the camera. Each blob is represented as a table containing position and color information.
function M.colored_blob_omnidirectional_camera.get_blobs()
	return robot.colored_blob_omnidirectional_camera
end

return M
