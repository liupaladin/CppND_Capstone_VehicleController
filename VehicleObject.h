#ifndef VEHICLEOBJECT_H
#define VEHICLEOBJECT_H

#include <string>
#include <iostream>

enum VehicleType {
    noObject,
    sedan,
    truck,
    suv,
};

class VehicleObject {
public:
    //constructor
    VehicleObject();

    //destructor
    ~VehicleObject();

    // setter
    void setMake(std::string make);
    void setType(VehicleType type);
    void setPhi(double phi) {_phi = phi;}
    void setPosition(double x, double y);
    void setTheta(double theta) { _theta = theta; }
    void initVelocity(double speed) { _speed = speed; }

    // getter
    std::string getMake() { return _make; }
    int getID() { return _id; }
    VehicleType getType() { return _type; }
    double getVelocity() { return _speed; }
    void getPosition(double &x, double &y);
    double getTheta() { return _theta; }

    // typical behavior methods
    virtual void simulate(double dt) {}; 
    // The dynamic update equation for each object which 
    // can be different for different vehicle type. dt is the discretization for discrete model


protected:
    std::string _make;   // store vehicle make
    int _id;             // identify the object unique id
    VehicleType _type;   // identify vehicle type
    double _speed;       // record vehicle speed
    double _x;           // record x coordinate of the object position
    double _y;           // record y coordinate of the object position
    double _theta;       // heading angle of the vehicle object
    double _phi;         // rate of change for steering angle at wheel

private:
    static int _idCnt; // global variable for counting object
    
};

#endif