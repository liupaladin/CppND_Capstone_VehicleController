#include <iostream>
#include "VehicleObject.h"

// init static variable
int VehicleObject::_idCnt = 0;

void VehicleObject::setPosition(double x, double y) {
    _x = x;
    _y = y;
}

void VehicleObject::getPosition(double &x, double &y) {
    x = _x;
    y = _y;
}
VehicleObject::VehicleObject() {
    _type = VehicleType::noObject;
    _id = _idCnt;
    _idCnt++;
}

VehicleObject::~VehicleObject() {
    std::cout << "Vehicle of address " << this << " is deleted\n";
}




