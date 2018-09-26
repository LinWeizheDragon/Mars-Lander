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
# Basic Autopilot Concept
## Two-dimensional automatic control system
Sometimes the aircraft needs to be speed up or slowed down to a specific velocity. However, it's not efficient at all to adjust the velocity in different dimensions one by one. So we need a two-dimensional automatic control system to adjust the velocity both vertically and horizontally at the same time.
### Throttle Setting and Attitude Change
The following code enable us to change speed in two-dimension
```
//set thrust
if (delta_r > 1 || delta_r < 0){
    cout<<"Delta value error: "<<delta_r<<endl;
    return;
}
if (Pout_r<=-delta_r){
    if (Pout_r <= -delta_r - 1){
        throttle1 = -1;
    }else{
        throttle1 = (Pout_r + delta_r);
    }
}
else if (Pout_r >= (1-delta_r)){
    throttle1 = 1;
}else{
    throttle1 = Pout_r + delta_r;
}

if (Pout_t >= 1){
    throttle2 = 1;
}else if (Pout_t<=0.1){
    throttle2 = 0;
}else{
    throttle2 = Pout_t;
}

vector3d new_attitude = v_t.norm() * throttle2  + e_r * throttle1;
attitude_autochange(new_attitude);
double new_throttle = sqrt(pow(throttle1,2) + pow(throttle2,2));
if (new_throttle >= 1) {
    throttle = 1;
}else{
    throttle = new_throttle;
}
```
# Scenarios
## Scenario 1
Please make sure:
```
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
```
### With the autopilot
The initial absolute velocity of the lander is zero but the ground speed is not due to the self rotation of the Mars. So we need a two-dimensional automatic control system to perform the landing
The program analyzes the power needed to slow down the aircraft in two-dimension, and then automatically changes the attitude of the lander and launches the engine.
(This is the core of landing/launc)
## Scenario 6
Please make sure:
```
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
```
### Without auto pilot
The lander runs on an aerostationary orbit.
### With auto pilot
The lander starts to slow down by activating the engine and pushing in opposite direction of velocity. 
After slowing down a specific speed, the engine is deactivated. 
The lander will follow an eclipse orbit to approach the atmosphere by the effect of gravitational force.
Then the engine is started again, slow down the lander and release the parachute.
Safely landed.

## Scenario 7


# Copyright
Lin Weizhe @2018
