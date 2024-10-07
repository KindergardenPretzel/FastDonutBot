#include "vex.h"
class DriveBase
{
    private:
        vex::inertial gyroSensor;
        vex::rotation fwdRotation;
        vex::rotation sideRotation;
        vex::motor MotorLF;
        vex::motor MotorLB;
        vex::motor MotorRF;
        vex::motor MotorRB;
        bool is_motor_reversed(int);

    public:
        vex::motor_group LeftMotors;
        vex::motor_group RightMotors;
        DriveBase(int, int, int, int, int, int, int);
};