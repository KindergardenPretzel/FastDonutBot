#include "vex.h"

class Odometry
{
    private:
        vex::inertial gyroSensor;
        vex::rotation fwdRotation;
        vex::rotation sideRotation;
    public:
        Odometry(int32_t fwdPort, int32_t sidePort,int32_t gyroPort);
};