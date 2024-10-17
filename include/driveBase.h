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
        void setRotation(double);
        float getHeading();
        void setHeading(double);
        double getHeadingRad();
        double getRotationRad();
        void resetFwdEncoder();
        void resetSideEncoder();
        float getFwdPosition();
        float getSidePosition();
        void SetBrake(vex::brakeType);
        void DriveDistance(float);
        void TurnAngle(float);
        float turnAngleOptimization(float);
};