#include "odometry.h"

Odometry::Odometry(int32_t fwdPort, int32_t sidePort,int32_t gyroPort) 
{
    gyroSensor = vex::inertial(gyroPort);
    fwdRotation = vex::rotation(fwdPort);
    sideRotation = vex::rotation(sidePort);
}