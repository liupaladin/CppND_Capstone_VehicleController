#ifndef PIDCTRLOBJECT_H
#define PIDCTRLOBJECT_H
#include <iostream>
#include "ControllerObject.h"

class PIDCtrlObject : public ControllerObject {
public:
    // contructor 
    PIDCtrlObject(double kp, double ki, double kd, double dt);

    // setter
    // set the PID gains for the controller respectively
    void setGain(double kp, double ki, double kd);

    // set the anti windup logic saturation limit
    void setAntiWindup(double AntiWindupLowerBound, double AntiWindupUpperBound);    

    // set the saturation limit for controller output
    void setSaturation(double minU, double maxU);

    // getter
    void getGain(double &kp, double &ki, double &kd);

    // typical behavior methods
    double getCtrlAction();

    // the method is used to update integral and derivative given the current error. 
    // the input newError represents the current error based on current measurement
    void updateError(double newError);

private: 
    double _kp;                     // proportional gain of the PID
    double _ki;                     // integral gain of the PID
    double _kd;                     // derivative gain of the PID
    double _integral;               // store integral of the error
    double _derivative;             // store derivative of the error
    double _AntiWindupUpperBound;   // upper bound of saturation for anti windup logic
    double _AntiWindupLowerBound;   // lower bound of saturation for anti windup logic
    double _maxU;                   // upper bound of the control output
    double _minU;                   // lower bound of the control output
};

#endif