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
        float default_drive_Kp{1.3};
        float default_drive_Ki{0};
        float default_drive_Kd{0};
        float default_drive_limit_integral{0.4};
        float default_drive_exit_error{0.3};
        float drive_default_min{1};
        float drive_default_max{9};
        int default_drive_timeout{5000};

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
        void DriveDistance(float distance);
        void DriveDistance(float distance, float heading);
        void DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout);
        void DriveDistance(float distance, float heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout);
        void DriveDistance(float distance, float dest_heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout);
        void TurnAngle(float);
        void swingRight(float);
        void swingLeft(float);
        float turnAngleOptimization(float);
};