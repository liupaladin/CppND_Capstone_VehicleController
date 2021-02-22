#include <iostream>
#include "PIDCtrlObject.h"

PIDCtrlObject::PIDCtrlObject(double kp, double ki, double kd, double dt) : ControllerObject(dt) {
    _type = CtrlMethod::PIDController;
    setGain(kp, ki, kd);
}

void PIDCtrlObject::setGain(double kp, double ki, double kd) {
    if (kp < 0) {
        _kp = 0.1;
    }
    _kp = kp;

    if (ki < 0) {
        _ki = 0.1;
    }
    _ki = ki;
    if (kd < 0) {
        _kd = 0.1;
    }
    _kd = kd;
}

void PIDCtrlObject::setAntiWindup(double AntiWindupLowerBound, double AntiWindUpperBound) {
    if (AntiWindUpperBound > AntiWindupLowerBound) {
        _AntiWindupUpperBound = AntiWindUpperBound;
        _AntiWindupLowerBound = AntiWindupLowerBound;
    }
}

void PIDCtrlObject::setSaturation(double minU, double maxU) {
    if (maxU > minU) {
        _minU = minU;
        _maxU = maxU;
    }
}

void PIDCtrlObject::getGain(double &kp, double &ki, double &kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PIDCtrlObject::updateError(double newError) {
    // update derivative of the error
    _derivative = (newError - _error) / _dt;

    // update the integral of the error and consider antiwindup
    if (_u >= _AntiWindupUpperBound || _u <= _AntiWindupLowerBound) {
        _integral += newError * _dt * 0;
    }
    else {
        _integral += newError * _dt;
    }

    // update the current error
    _error = newError;
}
double PIDCtrlObject::getCtrlAction() {
    _u = _kp * _error + _ki * _integral + _kd * _derivative;
    return _u;
}