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
        float inchesPerRev;
        bool is_motor_reversed(int);
        // odometry part
        const float SIDE_DISTANCE = 4;
        //const float FWD_DISTANCE = 0.6;
        const float FWD_DISTANCE = 0.5;

        float prev_heading;
        float x;
        float y;
        float fwdPosition;
        float sidePosition;
        float turnAngleOptimization(float);
        float backwardsAngleOptimization(float angle);
        void resetFwdEncoder();
        void resetSideEncoder();
        bool isLeftMotorSpinning();
        bool isRightMotorSpinning();
    public:
        float x1;
        float y1;

        float default_drive_Kp{1.5};
        float default_drive_Ki{0};
        float default_drive_Kd{8};
        float default_drive_limit_integral{1};
        float default_drive_exit_error{0.8};
        float default_drive_min{0};
        float default_drive_max{10};
        int default_drive_timeout{3000};

        float default_turn_Kp{0.4};
        float default_turn_Ki{0.03};
        float default_turn_Kd{3};
        float default_turn_limit_integral{15};
        float default_turn_exit_error{1};
        float default_turn_min{0};
        float default_turn_max{12};
        int default_turn_timeout{1500};

        float default_heading_Kp{0.4};
        float default_heading_Ki{0};
        float default_heading_Kd{1};
        float default_heading_limit_integral{0};
        float default_heading_exit_error{1};
        float default_heading_min{0};
        float default_heading_max{6};
        int default_heading_timeout{15000};
//odometry heading correction PID(0.4, 0, 1, 0, 1, 0, 10, 15000); 
        DriveBase(int gyroPort, int fwdRotatePort, int sideRotatePort, 
                     int MotorLFPort, int MotorLBPort, int MotorRFPort, 
                     int MotorRBPort, float inchesPerRev);
        vex::motor_group LeftMotors;
        vex::motor_group RightMotors;
        void calibrateInertial();
        float getHeading();
        void setHeading(double);
        float getFwdPosition();
        float getSidePosition();
        void SetBrake(vex::brakeType);
        void DriveDistance(float distance);
        void DriveDistance(float distance, float heading);
        void DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout);
        void DriveDistance(float distance, float heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout);
        void DriveDistance(float distance, float dest_heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout);
        void DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout);
        void TurnAngle(float angle, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout);
        void TurnAngle(float angle);
        void TurnAngle(float angle, float Kp, float Ki, float Kd);
        void swingRightHold(float);
        void swingLeftHold(float);

        void setStartingPoint(float, float, float);
        void updatePosition();
        float getX();
        float getY();
        void turnToXY(float x, float y);
        void driveStraightToXY(float destX, float destY); 
        //void driveToXY(float destX, float destY);
        void driveToXY(float destX, float destY, bool wait=true);
        void driveToXY(float destX, float destY, int timeout, bool wait=true);
        void driveToXY(float destX, float destY, float maxOut, int timeout, bool wait=false);
        bool isMoving();
        
        
       

};