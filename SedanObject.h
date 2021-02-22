#ifndef SEDAN_H
#define SEDAN_H

#include <string>
#include <iostream>
#include "VehicleObject.h"

class Sedan : public VehicleObject {
public:
    // constructor, destructor
    Sedan(double lf, double lr, double mass, double speed);

    // set the force input
    void setForce(double force) { _force = force; }

    // set the vehicle parameter for lf
    void setLf(double lf);

    // set the vehicle parameter for lr
    void setLr(double lr);

    // set the vehicle parameter for mass
    void setMass(double mass);

    // set the max steering angle of the wheel
    void setMaxSteer(double maxSteer) { _maxSteer = maxSteer; }

    // get the force measurement
    double getForce() { return _force; }

    // get the steering angle of the vehicle
    double getDelta() { return _delta; }

    // typical behavior methods
    void simulate(double dt);

private:
    
    
    double _force;      // the force that exerted on the vehicle
    double _delta;      // the steering angle at wheel    
    double _mass;       // parameter representing the mass of the sedan
    double _lf;         // parameter representing distance between CG and front axis
    double _lr;         // parameter representing distance between CD and rear axis
    double _maxSteer;   // the max wheel steer angle of the sedan, assume symetric for left and right most position

    /* 
    // Add controller by uncomment it out
    // define the PID controller for the vehicle longitudinal control
    PIDController PIDLong;

    // define Stanley controller for lane tracking
    StanleyController StanleyLat;
    */

    // typical behavior method
    void drive(double dt);

    // simulate the vehicle dynamics to get states updated
    // The dynamic equation used is a kinematic bicyle model
    // -----------------------------------------------------
    // xdot = v*cos(theta + beta), (represents vx)
    // ydot = v*sin(theta + beta), (represents vy)
    // vdot = F - 0.5*cd*A*v^2/ m,
    // thetaDot = v / lr * sin(beta)
    // deltaDot = phi
    // beta = arctan( lr / (lr + lf) * tan(delta))
    //
    // -----------------------------------------------------
    // use Euler integral varDot = var + varDot * dt
    // -----------------------------------------------------

};

#endif