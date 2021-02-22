#include <iostream>
#include "ControllerObject.h"

ControllerObject::ControllerObject(double dt) {
    _type = CtrlMethod::generic;
    if (dt > 0) {
        _dt = dt;
    }
    _u = 0;
}