#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "SedanObject.h"
#include "PIDCtrlObject.h"
#include "ControllerObject.h"

const double PI = 3.1415926;

int main() {
    double curPosX, curPosY;
    double dt = 0.1;
    double vTarget = 20;
    double headingTarget;
    double currError, ForceInput, steeringAngle;
    int simTime = 2000;
    // define vehicle 1
    Sedan sedan1(2,2.5,1500,0);
    sedan1.setPosition(0.0,0.0);
    sedan1.setTheta(0.0);
    sedan1.initVelocity(0.0);
    sedan1.setMaxSteer(PI / 6);
    
    // set up the speed controller
    PIDCtrlObject SpeedController(100, 1, 5, dt);
    SpeedController.setAntiWindup(-1000,1000);
    SpeedController.setSaturation(-2000, 2000);

    // set up the heading angle controller
    PIDCtrlObject headingController(0.02, 0.001, 0.2, dt);
    headingController.setAntiWindup(-0.01, 0.01);
    headingController.setSaturation(-0.015, 0.015);

    // set up file write to record results
    std::ofstream myfile;
    myfile.open("vehicle_data.txt");

    for (int i = 0; i < simTime; i++) {
        // set the force exerted on the vehicle to track target speed
        currError = vTarget - sedan1.getVelocity();
        SpeedController.updateError(currError);
        ForceInput = SpeedController.getCtrlAction();    
        sedan1.setForce(ForceInput);

        // Set the heading angle target for tracking
        headingTarget = sin(2 * PI / double(simTime) * i * 0.5);
        //headingTarget = 0.2;
        // set the steering rate to track target direction
        headingController.updateError(headingTarget - sedan1.getTheta());
        sedan1.setPhi(headingController.getCtrlAction());
        steeringAngle = sedan1.getDelta();

        // start simulation
        sedan1.simulate(dt);        
        sedan1.getPosition(curPosX, curPosY);

        // Print out the current status of the vehicle
        std::cout << "Current vehicle speed is " << sedan1.getVelocity() << "------------";
        std::cout << "position x = " << curPosX << ", position y = " << curPosY << "------------";
        std::cout << "Control Input Force is " << ForceInput << ",    ";
        std::cout << "Steering Angle is " << sedan1.getDelta() << ",    ";
        std::cout << "Heading Angle is " << sedan1.getTheta() << "\n";

        // write the vehicle states to the output file
        if (myfile.is_open()) {
            myfile << sedan1.getVelocity() << ", " << curPosX << ", " << curPosY << ", " << ForceInput << ", " << steeringAngle << ", " << sedan1.getTheta() << "\n";  
        }
    }
    myfile.close();
    return 0;
}