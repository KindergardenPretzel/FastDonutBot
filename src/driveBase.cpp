#include "vex.h"
#include "drivebase.h"
#include "PID.h"
#include <iostream>

//checks if the port is negative and returns true if it is
bool DriveBase::is_motor_reversed(int motor){
    if(motor - 1  < 0)
    {
        return true;
    }
    return false;
}

//DriveBase class constructor
DriveBase::DriveBase(int gyroPort, int fwdRotatePort, int sideRotatePort, 
                     int MotorLFPort, int MotorLBPort, int MotorRFPort, 
                     int MotorRBPort, float inchesPerRev):
 fwdRotation(abs(fwdRotatePort)), 
 sideRotation(abs(sideRotatePort)),
 gyroSensor(gyroPort),
 MotorLF(abs(MotorLFPort), vex::ratio6_1, is_motor_reversed(MotorLFPort)),
 MotorLB(abs(MotorLBPort), vex::ratio6_1, is_motor_reversed(MotorLBPort)),
 MotorRF(abs(MotorRFPort), vex::ratio6_1, is_motor_reversed(MotorRFPort)),
 MotorRB(abs(MotorRBPort), vex::ratio6_1, is_motor_reversed(MotorRBPort)),
 LeftMotors(vex::motor_group(MotorLF, MotorLB)), 
 RightMotors(vex::motor_group(MotorRF, MotorRB)),
 inchesPerRev(inchesPerRev)
{
    this->fwdRotation.setReversed(is_motor_reversed(fwdRotatePort));
    this->sideRotation.setReversed(is_motor_reversed(sideRotatePort));

}

//sets the starting point and heading of the robot. Resets Rotations sensors
void DriveBase::setStartingPoint(float startX, float startY, float startHeading ){
    this->x = startX;
    this->y = startY;
    // test part
    this->x1 = startX;
    this->y1 = startY;
    // test part
    this->setHeading(startHeading);
    this->prev_heading = startHeading;
    this->resetFwdEncoder();
    this->resetSideEncoder();
    this->fwdPosition = 0;
    this->sidePosition = 0;
}




//Odometry updates X and Y the position on the field of the robot
void DriveBase::updatePosition() {
    // getting current positions and current roatations and saving them into Class variables
    float fwdPos = this->getFwdPosition();
    float sidePos = this->getSidePosition();
    float currentHead  = toolbox::fround(this->getHeading());
    
    //calculating deltas(difference between old and new positions)
    float deltaFwd = fwdPos - this->fwdPosition;
    float deltaSide = sidePos - this->sidePosition;
    float deltaHead = currentHead - this->prev_heading;

    // convert heading difference to Radians
    float deltaHeadRad = toolbox::degreesToRadians(deltaHead);

    float localX;
    float localY;

    //calculating distance between robot tracking center
    if (deltaHead==0) {
        // if angle is not changed - then localY and localX is corresponding tracking wheel difference
        localX = deltaSide;
        localY = deltaFwd;
    }
    else{
    // calculate arc chord (h) for side movement and forward movement
     localX = 2 * sin(deltaHeadRad/2) * ( (deltaSide/deltaHeadRad) - SIDE_DISTANCE);
     localY = 2 * sin(deltaHeadRad/2) * ( (deltaFwd/deltaHeadRad) - FWD_DISTANCE);
     }

    
    // converting localX and localY to polar coordinates
    float vector_length;
    float angle_to_vector;

    if (localX == 0 && localY == 0) {
        vector_length = 0;
        angle_to_vector = 0;
    }
    else {
        // polar vector length is hypotenuse
        vector_length = sqrt(pow(localX,2) + pow(localY, 2));
        // (M_PI/2 - angle between vector and X axis) to calculate angle between Y and robot heading
        //angle_to_vector = M_PI - atan2(localY, localX);
        angle_to_vector = M_PI/2 - atan2(localY, localX);
    }
    // calculate new global angle and convert back to cartesian: x = r cos θ , y = r sin θ
    float global_angle_polar_coordinates = angle_to_vector - toolbox::degreesToRadians(this->prev_heading) - deltaHeadRad/2 ;

    float deltaX = vector_length * cos(global_angle_polar_coordinates);
    float deltaY = vector_length * sin(global_angle_polar_coordinates);

    // updating global X and Y by adding delta
   this->x += deltaX;
   this->y += deltaY;


///// test part
    float delta_Y_test = localY * cos(toolbox::degreesToRadians(this->prev_heading + deltaHead/2));
    float delta_X_test = localX * sin(toolbox::degreesToRadians(this->prev_heading + deltaHead/2));

    this->x1 += delta_X_test;
    this->y1 += delta_Y_test;
///// test part


    // updating stored positions and heading
    this->fwdPosition = fwdPos;
    this->sidePosition = sidePos;
    this->prev_heading = currentHead;
}

float DriveBase::getX() {
    return this->x;
}

float DriveBase::getY() {
    return this->y;
}


//calibrates the inertial
void DriveBase::calibrateInertial() {
    this->gyroSensor.calibrate();
    while (this->gyroSensor.isCalibrating())
    {
        vex::wait(20, vex::msec);
    };
}

//returns inertial sensor rotation [-180:180]
double  DriveBase::getRotation(){
    return this->gyroSensor.rotation();
}

//sets inertial sensor rotation
void DriveBase::setRotation(double value){
    this->gyroSensor.setRotation(value, vex::deg);
}

//returns inertial sensor absolute heading [0:360]
float DriveBase::getHeading(){
    return this->gyroSensor.heading();
}

//sets inertial sensor heading
void DriveBase::setHeading(double value){
    this->gyroSensor.setHeading(value, vex::deg);
}

//resets forward tracking sensor to 0
void DriveBase::resetFwdEncoder(){
     this->fwdRotation.resetPosition();
}

//resets side tracking sensor to 0
void DriveBase::resetSideEncoder(){
     this->sideRotation.resetPosition();
}

//returns current position of forward tracking sensoir in inches
float DriveBase::getFwdPosition(){
    return this->fwdRotation.position(vex::rev) * this->inchesPerRev;
     //return toolbox::fround(this->fwdRotation.position(vex::rev)) * this->inchesPerRev;
}

//returns current position of side tracking sensoir in inches
float DriveBase::getSidePosition(){
    return this->sideRotation.position(vex::rev) * this->inchesPerRev;
    //return toolbox::fround(this->sideRotation.position(vex::rev)) * this->inchesPerRev;
}

//sets all motor brake-types to the type instucted
void DriveBase::SetBrake(vex::brakeType brake_type) {
    this->MotorLF.setBrake(brake_type);
    this->MotorLB.setBrake(brake_type);
    this->MotorRF.setBrake(brake_type);
    this->MotorRB.setBrake(brake_type);

}

//optimizes turning for PID so it turns to the opposite side if the number is greater than 180
float DriveBase::turnAngleOptimization(float angle)
{
    if(!(angle > -180 && angle < 180))
    {
        if(angle < -180)
        {
            return angle += 360;
        }
        if(angle > 180)
        {
            return angle -= 360;
        }
    }
    return angle;
}

//optimizes turn angle for driving. In case of robot has to reverse heading it will drive backwards instead.
float DriveBase::backwardsAngleOptimization(float angle)
{
    if(!(angle > -90 && angle < 90))
    {
        if(angle < -90)
        {
            return angle += 180;
        }
        if(angle > 90)
        {
            return angle -= 180;
        }
    }
    return angle;
}

// drive function overloads. I hate to write them.
void DriveBase::DriveDistance(float distance){
    this->DriveDistance(distance, this->getHeading(), default_drive_Kp, default_drive_Ki, default_drive_Kd, default_drive_limit_integral, default_drive_exit_error, default_drive_min, default_drive_max, default_drive_timeout);
}

void DriveBase::DriveDistance(float distance, float heading){
    this->DriveDistance(distance, heading, default_drive_Kp, default_drive_Ki, default_drive_Kd, default_drive_limit_integral, default_drive_exit_error, default_drive_min, default_drive_max, default_drive_timeout);
}

void DriveBase::DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout){
    this->DriveDistance(distance, this->getHeading(), Kp, Ki, Kd, limit_integral, exit_error, default_drive_min, default_drive_max, timeout);
}

void DriveBase::DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout){
    this->DriveDistance(distance, this->getHeading(), Kp, Ki, Kd, limit_integral, exit_error, minOut, maxOut, timeout);
}

void DriveBase::DriveDistance(float distance, float heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout){
    this->DriveDistance(distance, heading, Kp, Ki, Kd, limit_integral, exit_error, default_drive_min, default_drive_max, timeout);
};

//robot drives forward or backward for the distance that is instructed
void DriveBase::DriveDistance(float distance, float dest_heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout){
    // define PID controllers for driving and heading correction
    PID drive_pid = PID(Kp, Ki, Kd, limit_integral, exit_error, minOut, maxOut, timeout);
    drive_pid.setDebug(true);
    PID heading_pid = PID(default_heading_Kp, default_heading_Ki, default_heading_Kd, default_heading_limit_integral, default_heading_exit_error, default_heading_min, default_heading_max, default_heading_timeout);

    // calculate destination by adding requested distance to current forward tracking wheel position in inches
    float destination = this->getFwdPosition() + distance;
    float error;
    float speed;
    float position;
    float heading_error;
    float heading_correction_speed;

    // if heading is specified in negative, convert to global heading [0 - 360]
    if (dest_heading < 0) { dest_heading += 360; };

    do
    {   
        // calculate distance to go
        position = this->getFwdPosition();
        error = destination - position;
        // calculate speed
        speed = drive_pid.calculate(error);

        // calculate heading correction angle and speed
        heading_error = dest_heading - this->getHeading();
        heading_correction_speed = heading_pid.calculate(turnAngleOptimization(heading_error));

        this->RightMotors.spin(vex::fwd, speed - heading_correction_speed, vex::volt);
        this->LeftMotors.spin(vex::fwd, speed + heading_correction_speed, vex::volt);
        vex::wait(20, vex::msec);

    }while(!drive_pid.isFinished());
    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);
}

void DriveBase::TurnAngle(float angle){
    this->TurnAngle(angle, default_turn_Kp, default_turn_Ki, default_turn_Kd, default_turn_limit_integral, default_turn_exit_error, default_turn_min, default_turn_max, default_turn_timeout);
}

void DriveBase::TurnAngle(float angle, float Kp, float Ki, float Kd){
    this->TurnAngle(angle, Kp, Ki, Kd, default_turn_limit_integral, default_turn_exit_error, default_turn_min, default_turn_max, default_turn_timeout);
}

//turns to the angle instructed [0:359]
void DriveBase::TurnAngle(float angle, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout)
{
    PID pid = PID(Kp, Ki, Kd, limit_integral, exit_error, minOut, maxOut, timeout);
    float error;
    float speed;
    float current_heading;
    if (angle < 0)
    {
        angle += 360;
    }
    do{
        current_heading = this->getHeading();
        error = this->turnAngleOptimization(angle - current_heading);
        speed = pid.calculate(error);
        this->LeftMotors.spin(vex::fwd, speed, vex::volt);
        this->RightMotors.spin(vex::fwd, -speed, vex::volt);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());

    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);
}

void DriveBase::swingRightHold(float angle)
{
    PID pid = PID(0.3, 0.001, 2, 15, 1, 3000);
    pid.setPIDmax(12);
    pid.setPIDmin(0);
    float error;
    float speed;
    float current_heading;
    do{
        current_heading = this->getHeading();
        error = this->turnAngleOptimization(angle - current_heading);
        //error = angle - current_heading;
        speed = pid.calculate(error);
        this->LeftMotors.spin(vex::fwd, speed, vex::volt);
        this->RightMotors.stop(vex::hold);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());

    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);
}

void DriveBase::swingLeftHold(float angle)
{
    PID pid = PID(0.3, 0.001, 2, 15, 1, 3000);
    pid.setPIDmax(12);
    pid.setPIDmin(0);
    float error;
    float speed;
    float current_heading;
    do{
        current_heading = this->getHeading();
        error = this->turnAngleOptimization(angle - current_heading);
        speed = pid.calculate(error);
        this->RightMotors.spin(vex::fwd, -speed, vex::volt);
        this->LeftMotors.stop(vex::hold);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());

    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);
}

void DriveBase::turnToXY(float destX, float destY)
{
    float currHead = this->getHeading();
    float currX = this->getX();
    float currY = this->getY();
    float angle_to_turn = toolbox::radiansToDegrees(atan2(destX - currX, destY - currY));
    this->TurnAngle(angle_to_turn);
}   

void DriveBase::driveStraightToXY(float destX, float destY)
{
float distance_to_drive = sqrt(pow(destX-this->getX(),2) + pow(destY-this->getY(),2));
    float currHead = this->getHeading();
    float currX = this->getX();
    float currY = this->getY();
    float angle_to_turn = toolbox::radiansToDegrees(atan2(destX - currX, destY - currY));  
    this->TurnAngle(angle_to_turn);
    this->DriveDistance(distance_to_drive,angle_to_turn);
}

// Odometry based drive to X, Y coordinate
void DriveBase::driveToXY(float destX, float destY)
{
    float error;
    float speed;
    float headingError;
    float headingCorrectionSpeed;
    float destHeading;
    float currX;
    float currY;

    // define PID controllers for Drive and Heading correction
    PID drive_pid = PID(default_drive_Kp, default_drive_Ki, default_drive_Kd, default_drive_limit_integral, default_drive_exit_error, default_drive_min, default_drive_max, default_drive_timeout);
    PID heading_pid = PID(default_heading_Kp, default_heading_Ki, default_heading_Kd, default_heading_limit_integral, default_heading_exit_error, default_heading_min, default_heading_max, default_heading_timeout);

    do
    {   // get current X, Y postion
        currX = this->getX();
        currY = this->getY();

        // Calculate distance to the point using pythagorean theorem.
        error = sqrt(pow(destX-currX,2) + pow(destY-currY,2));
        speed = drive_pid.calculate(error);

        // calculate heading correction angle using atan2 function. X,Y flipped, so we calculating angle to Y axis
        destHeading = toolbox::radiansToDegrees(atan2(destX - currX, destY - currY));
        headingError = destHeading - this->getHeading();

        float optimizedAngle = turnAngleOptimization(headingError);

        // if robot need to turn more than 90 degress - we going to go backwards. Cosine of angle more than 90 is negative.
        // we use it for reversing direction  
        float direction = cos(toolbox::degreesToRadians(optimizedAngle))/fabs(cos(toolbox::degreesToRadians(optimizedAngle)));
        
        // if we need to turn more than 90 degree - change angle and go backwards.
        headingCorrectionSpeed = heading_pid.calculate(backwardsAngleOptimization(optimizedAngle));

        this->RightMotors.spin(vex::fwd, direction * (speed - direction * headingCorrectionSpeed), vex::volt);
        this->LeftMotors.spin(vex::fwd,  direction * (speed + direction * headingCorrectionSpeed), vex::volt);
        vex::wait(20, vex::msec);

    }while(!drive_pid.isFinished());
    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);

}

