#include "odometry.h"

Odometry::Odometry(int fwdPort, int sidePort, int gyroPort, float in_per_rev):  fwdRotation(fwdPort), sideRotation(sidePort), gyroSensor(gyroPort), in_per_rev(in_per_rev)
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

void Odometry::setStartingPoint(float x, float y, float heading ){
    this->x = x;
    this->y = y;
    this->gyroSensor.setHeading(heading, vex::deg);
    this->fwdRotation.resetPosition();
    this->sideRotation.resetPosition();
    this->fwdPosition = 0;
    this->sidePosition = 0;

}

float Odometry::degreesToRadians(float degrees){
    return (degrees * M_PI) / 180;
}

void Odometry::updatePosition() {

    // getting current positions and current roatations
    float fwdPos = this->fwdRotation.position(vex::rev) * this->in_per_rev;
    float sidePos = this->sideRotation.position(vex::rev) * this->in_per_rev;
    float currentHead  = this->getHeading();
    //calculating deltas
    float deltaFwd = fwdPos - this->fwdPosition;
    float deltaSide = sidePos - this->sidePosition;
    float deltaHead = currentHead - this->heading;

    float deltaHeadRad = this->degreesToRadians(deltaHead);
    //calculating robot center path
    float h = 2 * sin(deltaHeadRad/2) * (deltaFwd/deltaHeadRad + 0.6);

    // updating stored positions and rotation
    this->fwdPosition = fwdPos;
    this->sidePosition = sidePos;
    this->heading = currentHead;
}