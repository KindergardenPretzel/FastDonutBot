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
        float default_drive_Kp{1.5};
        float default_drive_Ki{0};
        float default_drive_Kd{8};
        float default_drive_limit_integral{1};
        float default_drive_exit_error{0.8};
        float drive_default_min{0};
        float drive_default_max{10};
        int default_drive_timeout{2000};

        float default_turn_Kp{0.4};
        float default_turn_Ki{0.03};
        float default_turn_Kd{3};
        float default_turn_limit_integral{15};
        float default_turn_exit_error{1};
        float default_turn_min{0};
        float default_turn_max{12};
        int default_turn_timeout{5000};
        
        // odometry part
        const float SIDE_DISTANCE = 3;
        const float FWD_DISTANCE = 0.6;

        float prev_heading;
        float x;
        float y;
        float fwdPosition;
        float sidePosition;

    public:
        DriveBase(int, int, int, int, int, int, int, float);
        vex::motor_group LeftMotors;
        vex::motor_group RightMotors;
        void calibrateInertial();
        double getRotation();
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
        void TurnAngle(float angle, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout);
        void TurnAngle(float angle);
        void TurnAngle(float angle, float Kp, float Ki, float Kd);
        void swingRightHold(float);
        void swingLeftHold(float);
        float turnAngleOptimization(float);
        void setStartingPoint(float, float, float);
        void updatePosition();
        float getX();
        float getY();
        void turnToXY(float x,float y);

};