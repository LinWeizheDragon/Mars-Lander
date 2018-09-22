// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

bool integration_mode = 1;
int pilot_period = 0;

#define pi 3.1415926


void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
    if (scenario == 0){
        throttle = 1;
        
    }
    if (scenario == 1){
        //unit vector along radius
        vector3d e_r = vector3d(0,-1,0);
        
        static double Kh, Kp, error, h, Pout, delta;
        //get altitude
        h = position.abs() - MARS_RADIUS;
        
        //value setup
        Kh = 0.1;
        Kp = 1;
        
        //calculate error and Power
        error = -(0.5 + Kh * h + e_r * velocity);
        Pout = Kp * error;
        //cout<<(e_r * velocity)<<"   "<<(0.5 + Kh*h) <<endl;
        //cout<<"ERROR: "<<error<<endl;
        
        //r3 is relative distance to the power of 3
        double r3 = pow(position.abs(),3);
        
        //calculate gravitational force constant and mass
        double constant = GRAVITY * MARS_MASS / r3;
        double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
        
        //calculate gravitational force at this time
        double gravitational_force = mass * (position * constant * -1).abs();
        
        //setup delta value
        delta = gravitational_force / MAX_THRUST;
        //cout<<"Delta: "<<delta<<endl;
        
        //set thrust
        if (delta > 1 || delta < 0){
            cout<<"Delta value error: "<<delta<<endl;
            return;
        }
        if (Pout<=-delta){
            throttle = 0;
        }else if (Pout >= (1-delta)){
            throttle = 1;
        }else{
            throttle = Pout + delta;
        }
        
        //when throttle set and speed is lowered, release the parachute
        if (throttle>0 && velocity.abs()<=MAX_PARACHUTE_SPEED){
            parachute_status = DEPLOYED;
        }
        cout<<throttle<<endl;
    }
    
    
    
    if (scenario == 6){
        //orbital re-entry
        //unit vector along radius
        vector3d e_r = position.norm();
        //velocity along radius
        vector3d v_r = e_r * (e_r * velocity);
        //velocity along surface
        vector3d v_t = velocity - v_r;
        //cout<<v_r<<"    "<<v_t<<endl;
        
        static double a,c,launch_velocity;
        static double Kh, Kp, error, error2, h, Pout, delta, Kv, Kp2, Pout2;
        
        if (pilot_period == 0){
            if (simulation_time == 0){
                a  = (position.abs() + MARS_RADIUS) / 2.0;
                c = a - MARS_RADIUS;
                launch_velocity = sqrt((a-c)* GRAVITY * MARS_MASS / (a*(a+c)));
            }
            
            error2 = -(launch_velocity - v_t.abs());
            if (abs(error2)<=0.1){
                pilot_period++;
            }
            Pout2  = Kp2 * error2;
            Kp2 = 1;
            double throttle1,throttle2;
            
            if (Pout2 >= 1){
                throttle2 = 1;
            }else if (Pout2<=0.0){
                throttle2 = 0;
            }else{
                throttle2 = Pout2;
            }
            vector3d new_attitude = v_t.norm() * throttle2 * -1 + e_r * throttle1;
            attitude_autochange(new_attitude);
            throttle = throttle2;
            
            
        }else if (pilot_period == 1){
            //doing nothing
            if (position.abs()<= (MARS_RADIUS * 1.01)){
                pilot_period++;
            }
        }else if (pilot_period == 2){
            //get altitude
            h = position.abs() - MARS_RADIUS;
            
            //value setup
            Kh = 0.1;
            Kp = 0.5;
            if (simulation_time == 0){
                Kv = v_t.abs() / h;
            }
            
            Kp2 = 1;
            
            
            
            //calculate error and Power
            error = -(0.5 + Kh * h + e_r * velocity);
            
            Pout = Kp * error;
            //cout<<(e_r * velocity)<<"   "<<(0.5 + Kh*h) <<endl;
            //cout<<"ERROR: "<<error<<endl;
            
            //r3 is relative distance to the power of 3
            double r3 = pow(position.abs(),3);
            
            //calculate gravitational force constant and mass
            double constant = GRAVITY * MARS_MASS / r3;
            double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
            
            //calculate gravitational force at this time
            double gravitational_force = mass * (position * constant * -1).abs();
            
            
            //calculate error 2 and power
            error2 = -(Kv * h - v_t.abs()+10);
            Pout2  = Kp2 * error2;
            //cout<<Pout<<"    "<<Pout2<<endl;
            
            
            
            
            //setup delta value
            delta = gravitational_force / MAX_THRUST;
            
            
            
            double throttle1,throttle2;
            
            //set thrust
            if (delta > 1 || delta < 0){
                cout<<"Delta value error: "<<delta<<endl;
                return;
            }
            if (Pout<=-delta){
                throttle1 = 0;
            }else if (Pout >= (1-delta)){
                throttle1 = 1;
            }else{
                throttle1 = Pout + delta;
            }
            
            if (Pout2 >= 1){
                throttle2 = 1;
            }else if (Pout2<=0.1){
                throttle2 = 0;
            }else{
                throttle2 = Pout2;
            }
            vector3d new_attitude = v_t.norm() * throttle2 * -1 + e_r * throttle1;
            attitude_autochange(new_attitude);
            double new_throttle = sqrt(pow(throttle1,2) + pow(throttle2,2));
            if (new_throttle >= 1) {
                throttle = 1;
            }else{
                throttle = new_throttle;
            }
            //when throttle set and speed is lowered, release the parachute
            if (throttle>0 && velocity.abs()<=MAX_PARACHUTE_SPEED){
                parachute_status = DEPLOYED;
            }
        }
        
        
    }
}

vector3d getAcceleration (vector3d pureAcceleration)
/*
 This function add aerodynamic drag force and thrust to the pure acceleration calculated.
 */
{
    //calculate mass by fuel remaining and mass when unloaded
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    
    //get the thrust from predefined function
    vector3d thr = thrust_wrt_world();
    if (thr.abs()!=0){
        //cout<<thr<<endl;
    }
    
    //calculate the aerodynamic drag force
    vector3d aero_drag;
    double density = atmospheric_density(position);
    aero_drag = velocity.norm() * -0.5 * density * DRAG_COEF_LANDER * pi * pow(LANDER_SIZE,2) * pow (velocity.abs(),2);
    //cout<<aero_drag<<"     ";
    //when parachute is deployed, add more drag force
    if (parachute_status == DEPLOYED){
        aero_drag += velocity.norm() * -0.5 * density * DRAG_COEF_CHUTE * 5 * pow((LANDER_SIZE*2.0),2) * pow (velocity.abs(),2);
    }
    //cout<<aero_drag<<endl;
    
    vector3d new_acceleration = pureAcceleration + thr / mass + aero_drag / mass;
    return new_acceleration;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    //declare a static variable to store previous position.
    static vector3d previous_position;
    static vector3d acceleration;
    
  // INSERT YOUR CODE HERE
    
    //r3 is relative distance to the power of 3
    double r3 = pow(position.abs(),3);
    
    //calculate gravitational force constant
    double constant = GRAVITY * MARS_MASS / r3;
    
    
    
    
    
    
    if (integration_mode == MODE_EULER){
        //Euler Integration
        //work out acceleration
        acceleration = position * constant * -1;
        acceleration = getAcceleration(acceleration);
        //update position and velocity
        position = position + (velocity * delta_t);
        velocity = velocity + (acceleration * delta_t);
        
    }
    else if (integration_mode == MODE_VERLET){
        // Verlet integration
        //cout<<orientation<<endl;
        if (simulation_time == 0){
            //first step
            //init the acceleration and velocity.
            previous_position = position;
            acceleration = position * constant * -1;
            acceleration = getAcceleration(acceleration);
            position = position + velocity * delta_t + acceleration * 0.5 *pow(delta_t,2);
            
        }else{
            //use a variable to store previous position
            vector3d mid_position;
            mid_position = position;
            position = position * 2 - previous_position + acceleration * delta_t * delta_t;
            velocity = (position - previous_position) * 0.5 / delta_t;
            
            //update acceleration
            acceleration = position * constant * -1;
            acceleration = getAcceleration(acceleration);
            //update previous position
            previous_position = mid_position;
        }
        
        
    }
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "orbital re-entry";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";
    
    integration_mode = MODE_VERLET;
  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
          position = vector3d(MARS_RADIUS + 17032000, 0.0, 0.0);
          velocity = vector3d(0.0, sqrt(GRAVITY * MARS_MASS / position.abs()),0.0);
          orientation = vector3d(0.0, 90.0, 0.0);
          delta_t = 0.1;
          parachute_status = NOT_DEPLOYED;
          stabilized_attitude = false;
          autopilot_enabled = true;
          pilot_period = 0;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
