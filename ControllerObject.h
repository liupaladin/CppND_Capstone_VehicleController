#ifndef CONTROLLEROBJECT_H
#define CONTROLLEROBJECT_H

#include <iostream>

enum CtrlMethod {
    generic,
    PIDController,
};

class ControllerObject {
public:
    ControllerObject(double dt);

    // setter
    void setError(double error) { _error = error; }
    
    // getter
    double getDt() { return _dt; }
    
    // typical behavior methods
    virtual double getCtrlAction(){};

protected:
    CtrlMethod _type; // identifies the controller type
    double _error;    // the feedback signal compared to target
    double _dt;       // the time discretization step for the controller to update.
    double _u = 0;        // the control command from the controller 
};

#endif