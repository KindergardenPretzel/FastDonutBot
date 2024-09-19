#include "odometry.h"

Odometry::Odometry(int fwdPort, int sidePort, int gyroPort):  fwdRotation(fwdPort), sideRotation(sidePort), gyroSensor(gyroPort)
{
};

void Odometry::calibrateInertial() {
    this->gyroSensor.calibrate();
    while (this->gyroSensor.isCalibrating())
    {
        vex::wait(20, vex::msec);
    };
}

double  Odometry::getRotation(){
    return this->gyroSensor.rotation();
}

void Odometry::setRotation(double value){
    this->gyroSensor.setRotation(value, vex::deg);
}

double  Odometry::getHeading(){
    return this->gyroSensor.heading();
}

void Odometry::setHeading(double value){
    this->gyroSensor.setHeading(value, vex::deg);
}

double Odometry::getHeadingRad(){
    double heading = this->gyroSensor.heading();
    return (heading * M_PI) / 180;
}

double Odometry::getRotationRad(){
    double rotation = this->gyroSensor.rotation();
    return (rotation * M_PI) / 180;
}

void Odometry::setStartingPoint(double x, double y){
    this->x = x;
    this->y = y;
}