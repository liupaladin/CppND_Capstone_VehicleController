#include <iostream>
#include <math.h>
#include <assert.h>
#include "SedanObject.h"

Sedan::Sedan(double lf, double lr, double mass, double speed) : VehicleObject(){
    setLf(lf);
    setLr(lr);
    setMass(mass);   
    _speed = speed; //m/s
}

void Sedan::setLf(double lf) {
    if (lf <= 0) {
        // can add exception handler later
        lf = 1;
    }
    _lf = lf;
}

void Sedan::setLr(double lr) {
    if (lr <= 0) {
        // can add exception handler later
        lr = 1;
    }
    _lr = lr;
}

void Sedan::setMass(double mass) {
    if (mass <= 0) {
        // can add exception handler later
        mass = 1000;
    }
    _mass = mass;
}

void Sedan::simulate(double dt) {
    drive(dt);
}

void Sedan::drive(double dt) {
    // simulate the vehicle dynamics to get states updated
    // The dynamic equation used is a kinematic bicyle model
    // -----------------------------------------------------
    // xdot = v*cos(theta + beta), (represents vx)
    // ydot = v*sin(theta + beta), (represents vy)
    // vdot = F / m,
    // thetaDot = v / lr * sin(beta)
    // deltaDot = phi
    // beta = arctan( lr / (lr + lf) * tan(delta))
    //
    // -----------------------------------------------------
    // use Euler integral varDot = var + varDot * dt
    // -----------------------------------------------------
    double xdot, ydot, vdot, thetaDot, deltaDot, beta;
    beta = atan( _lr / (_lr + _lf) * tan(_delta) );
    xdot = _speed * cos(_theta + beta);
    ydot = _speed * sin(_theta + beta);
    vdot = (_force - 0.5 * 3 * 0.4 * _speed*_speed)/ _mass;
    thetaDot = _speed / _lr * sin(beta);
    deltaDot = _phi;

    _x += xdot * dt;
    _y += ydot * dt;
    _speed += vdot * dt;
    _theta += thetaDot * dt;

    _delta += deltaDot * dt;  
    if (_delta >= _maxSteer) {
        _delta = _maxSteer;
    }
    else if (_delta <= -_maxSteer) {
        _delta = -_maxSteer;
    }
}