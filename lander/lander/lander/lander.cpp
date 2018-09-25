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

bool integration_mode = MODE_VERLET; // 1 for Verlet integration, 0 for Euler integration. See definition in .h
int pilot_period = 0; //variable to store pilot period

//settings for scenario 1
#define MODE_1_LANDING 0
#define MODE_1_PRE_INJECTION 1
#define MODE_1_INJECTION 2
#define MODE_5_PRE_INJECTION 1
#define MODE_5_INJECTION 2

int action_mode = MODE_1_PRE_INJECTION;

// settings for orbital injection
// apogee > perigee
double injection_orbit_apogee = (MARS_RADIUS + 17032000) * 1.3;
double injection_orbit_perigee = MARS_RADIUS + 17032000;

//constant

#define STABLE_COUNT_CHECK 100


//function definition
void autopilot_orbital_injection();
void autopilot_orbital_pre_injection(double to_height, double to_v_t, double to_v_r);

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
    if (scenario == 0){
        //unit vector along radius
        vector3d e_r = position.norm();
        //velocity along radius
        vector3d v_r = e_r * (e_r * velocity);
        //velocity along surface
        vector3d v_t = velocity - v_r;
        cout<<"v_r"<<v_r<<endl<<"v_t"<<v_t<<endl;
        
    }
    if (scenario == 1){
        static double Kh, Kp, error, error2, h, Pout, delta, Kv, Kp2, Pout2;
        
        //get altitude
        //unit vector along radius
        vector3d e_r = position.norm();
        //velocity along radius
        vector3d v_r = e_r * (e_r * getRelativeVelocity(velocity));
        //velocity along surface
        vector3d v_t = getRelativeVelocity(velocity) - v_r;
        h = position.abs() - MARS_RADIUS;
        
        //value setup
        Kh = 0.1;
        Kp = 0.5;
        if (simulation_time == 0){
            Kv = v_t.abs() / h;
        }
        
        Kp2 = 1;
        
        
        
        //calculate error and Power
        error = -(0.5 + Kh * h + e_r * getRelativeVelocity(velocity));
        
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
        double drag;
        drag = 0.5*DRAG_COEF_CHUTE*atmospheric_density(position)*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*velocity.abs2();
        if (throttle>0 && velocity.abs()<MAX_PARACHUTE_SPEED ){
            cout<<velocity.abs()<<endl;
            parachute_status = DEPLOYED;
        }
        
    }
    if (scenario == 3){
        autopilot_orbital_injection();
    }
    
    if (scenario==5){
        if (action_mode==MODE_5_PRE_INJECTION){
            double perigee = injection_orbit_perigee;
            double to_v_t = sqrt(GRAVITY * MARS_MASS*(1 / (MARS_RADIUS + EXOSPHERE) - 1 / perigee) * 2 / (1 - pow(MARS_RADIUS + EXOSPHERE,2)/pow(perigee,2)))/2;
            autopilot_orbital_pre_injection((EXOSPHERE+(position.abs()-MARS_RADIUS))/2, to_v_t, 2000);
        }else if(action_mode==MODE_5_INJECTION){
            autopilot_orbital_injection();
        }
    }
    
    if (scenario == 6){
        //orbital re-entry
        //#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
        
        
        static double a,c,launch_velocity;
        static double Kh, Kp, error, error2, h, Pout, delta, Kv, Kp2, Pout2;
        
        if (pilot_period == -1){
            //unit vector along radius
            vector3d e_r = position.norm();
            //velocity along radius
            vector3d v_r = e_r * (e_r * velocity);
            //velocity along surface
            vector3d v_t = velocity - v_r;
            //cout<<v_r<<"    "<<v_t<<endl;
            
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
            double throttle2;
            
            if (Pout2 >= 1){
                throttle2 = 1;
            }else if (Pout2<=0.0){
                throttle2 = 0;
            }else{
                throttle2 = Pout2;
            }
            vector3d new_attitude = v_t.norm() * -1;
            attitude_autochange(new_attitude);
            throttle = throttle2;
            
        }else if (pilot_period == 0){
            //doing nothing
            if (position.abs()<= (MARS_RADIUS * 1.005)){
                pilot_period++;
            }
        }else if (pilot_period == 1){
            //get altitude
            //unit vector along radius
            vector3d e_r = position.norm();
            //velocity along radius
            vector3d v_r = e_r * (e_r * getRelativeVelocity(velocity));
            //velocity along surface
            vector3d v_t = getRelativeVelocity(velocity) - v_r;
            h = position.abs() - MARS_RADIUS;
            
            //value setup
            Kh = 0.1;
            Kp = 0.5;
            if (simulation_time == 0){
                Kv = v_t.abs() / h;
            }
            
            Kp2 = 1;
            
            
            
            //calculate error and Power
            error = -(0.5 + Kh * h + e_r * getRelativeVelocity(velocity));
            
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
            double drag;
            drag = 0.5*DRAG_COEF_CHUTE*atmospheric_density(position)*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*velocity.abs2();
            if (throttle>0 && velocity.abs()<MAX_PARACHUTE_SPEED ){
                cout<<velocity.abs()<<endl;
                parachute_status = DEPLOYED;
            }
        }
        
        
    }
    if (scenario==7){
        if (action_mode==MODE_1_PRE_INJECTION){
            double perigee = injection_orbit_perigee;
            double to_v_t = sqrt(GRAVITY * MARS_MASS*(1 / (MARS_RADIUS + EXOSPHERE) - 1 / perigee) * 2 / (1 - pow(MARS_RADIUS + EXOSPHERE,2)/pow(perigee,2)))/2;
            autopilot_orbital_pre_injection((EXOSPHERE+(position.abs()-MARS_RADIUS))/2, to_v_t, 2000);
        }else if(action_mode==MODE_1_INJECTION){
            autopilot_orbital_injection();
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
    vector3d relative_velocity = getRelativeVelocity(velocity);
    
    //use relative speed to calculate the drag forces
    aero_drag = relative_velocity.norm() * -0.5 * density * DRAG_COEF_LANDER * pi * pow(LANDER_SIZE,2) * pow (relative_velocity.abs(),2);
    
    //when parachute is deployed, add more drag force
    if (parachute_status == DEPLOYED){
        aero_drag += relative_velocity.norm() * -0.5 * density * DRAG_COEF_CHUTE * 5 * pow((LANDER_SIZE*2.0),2) * pow (relative_velocity.abs(),2);
    }
    //cout<<aero_drag<<endl;
    
    vector3d new_acceleration = pureAcceleration + thr / mass + aero_drag / mass;
    return new_acceleration;
}
void autopilot_orbital_pre_injection(double to_height, double to_v_t, double to_v_r){
    //speed up aircraft
    //unit vector along radius
    vector3d e_r = position.norm();
    //velocity along radius
    vector3d v_r = e_r * (e_r * velocity);
    //velocity along surface
    vector3d v_t = velocity - v_r;
    
    //r3 is relative distance to the power of 3
    double r3 = pow(position.abs(),3);
    // get current height
    double h = position.abs() - MARS_RADIUS;
    //calculate gravitational force constant and mass
    double constant = GRAVITY * MARS_MASS / r3;
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    
    //calculate gravitational force at this time
    double gravitational_force = mass * (position * constant * -1).abs();
    // launch into orbit towards perigee
    static double Kh_r;
    double Kp_r, error_r, Pout_r;
    static double Kh_t;
    double Kp_t, error_t, Pout_t;
    static double start_v_t, start_v_r, start_height;
    
    
    if (pilot_period == -1){
        // initialize
        start_height = to_height;
        start_v_r = to_v_r;
        start_v_t = to_v_t;
        cout<<"settings:"<<start_height<<endl<<start_v_r<<endl<<start_v_t<<endl;
        Kh_r = (start_v_r - v_r.abs() / to_height - h);
        //cout<<"set start v_t "<<start_v_t<<endl;
        pilot_period++;
    }
    
    //Pout along radius
    Kp_r = 1.0;
    error_r = Kh_r * (h - start_height) - v_r * e_r;
    cout<<"error r:"<<error_r<<endl;
    Pout_r = Kp_r * error_r;
    //setup delta value
    float delta_r = gravitational_force / MAX_THRUST;
    
    //Pout parpendicular to radius
    Kh_t = 1.0;
    Kp_t = 1.0;
    error_t = -1 * Kh_t * (v_t.abs() - start_v_t);
    cout<<"error t:"<<error_t<<endl;
    Pout_t = Kp_t * error_t;
    
    double throttle1,throttle2;
    
    //set thrust
    
    if (delta_r > 1 || delta_r < 0){
        cout<<"Delta value error: "<<delta_r<<endl;
        return;
    }
    if (Pout_r<=-delta_r){
        throttle1 = 0;
    }else if (Pout_r >= (1-delta_r)){
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
    cout<<v_t.norm()<<endl;
    vector3d new_attitude = v_t.norm() * throttle2  + e_r * throttle1;
    attitude_autochange(new_attitude);
    double new_throttle = sqrt(pow(throttle1,2) + pow(throttle2,2));
    if (new_throttle >= 1) {
        throttle = 1;
    }else{
        throttle = new_throttle;
    }
    if (abs(error_t) < 1){
        pilot_period = -1;
        action_mode = MODE_1_INJECTION;
    }
}
void autopilot_orbital_injection(){
    
    //launch into orbit
    //#define FUEL_RATE_AT_MAX_THRUST 0.0 // (l/s)
    
    double apogee = injection_orbit_apogee;
    double perigee = injection_orbit_perigee;
    //unit vector along radius
    vector3d e_r = position.norm();
    //velocity along radius
    vector3d v_r = e_r * (e_r * velocity);
    //velocity along surface
    vector3d v_t = velocity - v_r;
    
    //r3 is relative distance to the power of 3
    double r3 = pow(position.abs(),3);
    // get current height
    double h = position.abs() - MARS_RADIUS;
    //calculate gravitational force constant and mass
    double constant = GRAVITY * MARS_MASS / r3;
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    
    //calculate gravitational force at this time
    double gravitational_force = mass * (position * constant * -1).abs();
    
    if (pilot_period == -1 || pilot_period == 0){
        // launch into orbit towards perigee
        static double Kh_r;
        double Kp_r, error_r, Pout_r;
        static double Kh_t;
        double Kp_t, error_t, Pout_t;
        static double start_v_t;
        
        
        if (pilot_period == -1){
            // initialize
            Kh_r = (v_r.abs() / EXOSPHERE);
            start_v_t = sqrt(GRAVITY * MARS_MASS*(1 / (MARS_RADIUS + EXOSPHERE) - 1 / perigee) * 2 / (1 - pow(MARS_RADIUS + EXOSPHERE,2)/pow(perigee,2)));
            cout<<"set start v_t "<<start_v_t<<endl;
            pilot_period++;
        }
        
        //Pout along radius
        Kp_r = 1.0;
        error_r = Kh_r * (EXOSPHERE - h) - v_r * e_r;
        cout<<"v_r.abs():"<<v_r * e_r<<"   "<<Kh_r * (EXOSPHERE - h)<<endl;
        cout<<"error r:"<<error_r<<endl;
        Pout_r = Kp_r * error_r;
        //setup delta value
        float delta_r = gravitational_force / MAX_THRUST;
        
        //Pout parpendicular to radius
        Kh_t = 1.0;
        Kp_t = 1.0;
        error_t = -1 * Kh_t * (v_t.abs() - start_v_t);
        cout<<"v_t.abs():"<<v_t.abs()<<"   "<<start_v_t<<endl;
        cout<<"error t:"<<error_t<<endl;
        Pout_t = Kp_t * error_t;
        
        double throttle1,throttle2;
        
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
        }else if (Pout_r >= (1-delta_r)){
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
        if (error_t < 1 && (v_r.abs())<10 && (EXOSPHERE - h) < 100){
            cout<<"final:"<<v_r.abs()<<"||"<<v_t.abs()<<"||"<<h<<endl;
            throttle = 0;
            pilot_period++;
        }
        
    }
    if (pilot_period==1){
        //doing nothing until reach perigee
        cout<<perigee - position.abs()<<endl;
        if (perigee - position.abs() < 100000){
            pilot_period++;
        }
    }
    if (pilot_period==2 || pilot_period==3){
        static int stable_count = 0;
        static double Kh_r;
        double Kp_r, error_r, Pout_r;
        static double Kh_t;
        double Kp_t, error_t, Pout_t;
        static double end_v_t;
        if (pilot_period == 2){
            //initialize
            Kh_r = abs(v_r.abs() / (perigee - position.abs()));
            if (perigee == apogee){
                end_v_t = sqrt(GRAVITY * MARS_MASS / perigee);
            }else{
                end_v_t = sqrt(GRAVITY * MARS_MASS*(1 / (perigee) - 1 / apogee) * 2 / (1 - pow(perigee,2)/pow(apogee,2)));
                cout<<"set end_v_t"<<end_v_t<<endl;
            }
            pilot_period++;
        }
        // Pout along radius
        Kp_r = 1;
        error_r = Kh_r * (perigee - position.abs()) - v_r * e_r;
        cout<<"error r:"<<error_r<<endl;
        Pout_r = Kp_r * error_r;
        //setup delta value
        double delta_r = gravitational_force / MAX_THRUST;
        
        //Pout parpendicular to radius
        Kh_t = 1.0;
        Kp_t = 1.0;
        error_t = -1 * Kh_t * (v_t.abs() - end_v_t);
        cout<<"error t:"<<error_t<<endl;
        Pout_t = Kp_t * error_t;
        
        double throttle1,throttle2;
        
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
        }else if (Pout_r >= (1-delta_r)){
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
        cout<<throttle1<< "===="<<throttle2<<endl;
        vector3d new_attitude = v_t.norm() * throttle2  + e_r * throttle1;
        attitude_autochange(new_attitude);
        double new_throttle = sqrt(pow(throttle1,2) + pow(throttle2,2));
        if (new_throttle >= 1) {
            throttle = 1;
        }else{
            throttle = new_throttle;
        }
        if (abs(error_t) < 1 && abs(error_r) < 1){
            stable_count++;
            if (stable_count > STABLE_COUNT_CHECK){
                //release
                throttle = 0;
                pilot_period++;
            }
            
        }
    }
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
  scenario_description[7] = "from rest at 10km altitude, launch into orbit";
  scenario_description[8] = "";
  scenario_description[9] = "";
    
    integration_mode = MODE_VERLET;
    pilot_period = -1;
    
  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
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
    autopilot_enabled = true;
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
    stabilized_attitude = false;
    autopilot_enabled = true;
    break;

  case 6:
          //orbital re-entry
          position = vector3d(MARS_RADIUS + 17032000, 0.0, 0.0);
          velocity = vector3d(0.0, sqrt(GRAVITY * MARS_MASS / position.abs()),0.0);
          orientation = vector3d(0.0, 90.0, 0.0);
          delta_t = 0.1;
          parachute_status = NOT_DEPLOYED;
          stabilized_attitude = false;
          autopilot_enabled = true;
    break;

  case 7:
          // a descent from rest at 10km altitude, launch into orbit
          position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
          velocity = vector3d(0.0, 0.0, 0.0);
          orientation = vector3d(0.0, 0.0, 90.0);
          delta_t = 0.1;
          parachute_status = NOT_DEPLOYED;
          stabilized_attitude = false;
          autopilot_enabled = true;
          break;

  case 8:
    break;

  case 9:
    break;

  }
}
