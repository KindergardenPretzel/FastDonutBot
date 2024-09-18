#include "odometry.h"

Odometry::Odometry(int fwdPort, int sidePort, int gyroPort):  fwdRotation(fwdPort), sideRotation(sidePort), gyroSensor(gyroPort)
{
};

void Odometry::calibrateInertial() {
    this->gyroSensor.startCalibration(2000);
    while (this->gyroSensor.isCalibrating())
    {
        vex::wait(20, vex::msec);
    };
}