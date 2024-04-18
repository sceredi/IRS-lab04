# IRS Fourth Activity
## Objective
The objective of this activity is the implementation of the phototaxis with collision avoidance behaviour, utilizing the
motor schemas control model.
### Constraints
- Utilize the motor shema control model for the implementation.
- Move towards the light as swiftly as possible.
- Avoid collision with obstacles (or other robots).
- Do not go over the speed of 15 cm/s on each wheel.

## Solution Implemented
### Motor Schema for phototaxis
The motor schema implemented for phototaxis works with a perceptual schema that involves all 24 light sensors of the robot.
The light sensors values and angles are considered as vectors with length -> value and angle -> angle and get summed toghether,
the resulting vector will be used in 2 different fields:
- An attraction field which takes the sum length and angle and creates an attraction field as follows:
```lua
local function attraction_field(strength, angle)
    local length = 0
    if strength > 0 then
        length = math.min(strength, S) / S
    else
        length = 0
    end
    return { length = length, angle = angle }
end
```
S is a constant that represents the maximum strength a field can have, all the strengths are rescaled into the range [0..1] for
ease of use.
- A uniform created as follows
```lua
local function uniform_field(strength, angle)
    local length = 0
    if strength > 0 then
        length = math.min(strength, S) / S
    else
        length = 0
    end
    return { length = length, angle = angle }
end
```
In this second field the strength also depends from the same perceptual schema, with a variation as this field gets used in
order to move the robot in a straight line when the light cannot be seen or is too far away, as shown after:
```lua
local go_straight = uniform_field(0.5 - light_polar_val.strength, 0)
```
This way it allows the robot to move forward when the light is too far away or hidden, while at the same time avoiding having
very sharp turns when avoiding obstacles and so maintaining the robot in a straighter trajectory towards the light source even
when far away from it.

### Motor Schema for obstacle avoidance
The motor schema for obstacle avoidance works with a similar perceptual schema as the phototaxis one which involves
all 24 proximity sensors with their value and angle considered as vectors and summed together, the resulting
vector will be used in 2 different fields also in this case:
- A repulsive field which takes the length and angle of the percieved obstacles and creates the field as follows:
```lua
local function repulsive_field(strength, angle)
    local length = 0
    if strength > 0 then
        length = math.min(strength, S) / S
    else
        length = 0
    end
    return { length = length, angle = -angle }
end
```
Allowing the robot to avoid obstacles from both directions and enbling it to go through corridors without
"zig-zagging" too much.
- A tangential field, created as follows:
```lua
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
```
That creates a field tangential to the obstacle allowing the robot to circumnavigate it instead of fleeing from it
and also making the turns away from the obstacles less sharp when far away from the light or in a place where it
is not visible. A bit of fine tuning of the field strength was required as a value too high may loop the robot around
the obstacle, while a value too low make not create a strong enough field. The value decided in for this field is
as follows:
```lua
local obstacle_tangential = tangential_field(obstacle_polar_val.strength * 2, obstacle_polar_val.angle)
```
In the end the vectors of all the fields get summed, transformed into the differential steering model and sent
to the wheels.

## Problems encountered
The biggest problem encountered with the implementation of motor schemas if the debugging difficulty as a fusion
of all the fields. For example initially the tangential field was always adding pi/2 to the angle, making the 
robot always turn left when it encountered an obstacle, even if the obstacle was on it's left side. In order to
find where the bug was the debugging process took quite some time and immagination. So for more complex systems
I can't imagine that a purely motor shemas implementation is the best option.
