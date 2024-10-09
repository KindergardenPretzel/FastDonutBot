#include "vex.h"
#include "toolbox.h"
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
        float in_per_rev;
        bool is_motor_reversed(int);

    public:
        DriveBase(int, int, int, int, int, int, int, float);
        vex::motor_group LeftMotors;
        vex::motor_group RightMotors;
        void calibrateInertial();
        double  getRotation();
        void setRotation(double value);
        double  getHeading();
        void setHeading(double value);
        double getHeadingRad();
        double getRotationRad();
        void resetFwdEncoder();
        void resetSideEncoder();
        float getFwdPosition();
        float getSidePosition();
        void SetBrake(vex::brakeType);
};