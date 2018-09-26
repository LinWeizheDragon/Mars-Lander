# Mars-Lander
My Mars lander

# Features
## Acceleration
### Using the following function to add drag force/wind flow/gusts to pure acceleration produced by gravitational force
```vector3d getRelativeVelocity(vector3d absolute_velocity);```
### Using the following function to obtain relative velocity (useful when trying to involve wind flow/gusts)
```vector3d getRelativeVelocity(vector3d absolute_velocity);```
## Wind Flow
### Wind Flow Control Constant (lander.h)
```
// Wind Flow Constant
#define WIND_FLOW_SPEED 1 // wind flow speed on ground (m/s)
// wind flow direction, two conponents of the direction vector
// eg: (1, 1) points to north east.
#define WIND_FLOW_DIRECTION_NORTH 1 //negative number indicates south
#define WIND_FLOW_DIRECTION_EAST 1 //negative number indicates west
#define WIND_FLOW_RANDOM_CONSTANT 1 //speed of additional random gusts (m/s)
```
## Switch Integration Mode
lander.h
```
#define MODE_VERLET 1
#define MODE_EULER 0
```
lander.cpp:
```
bool integration_mode = MODE_VERLET; 
```
# Scenarios
## Scenario 6
Please make sure:
```
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
```
### Without auto pilot
The lander runs on an aerostationary orbit.
### With auto pilot
The lander starts

# Copyright
Lin Weizhe @2018
