#include "vex.h"

class Odometry
{
    private:
        vex::inertial gyroSensor;
        vex::rotation fwdRotation;
        vex::rotation sideRotation;
    public:
        Odometry(int, int, int);
        void calibrateInertial();
};