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
                     int MotorRBPort, float in_per_rev):
 fwdRotation(abs(fwdRotatePort)), 
 sideRotation(abs(sideRotatePort)),
 gyroSensor(gyroPort),
 MotorLF(abs(MotorLFPort)),
 MotorLB(abs(MotorLBPort)),
 MotorRF(abs(MotorRFPort)),
 MotorRB(abs(MotorRBPort)),
 LeftMotors(vex::motor_group(MotorLF, MotorLB)), 
 RightMotors(vex::motor_group(MotorRF, MotorRB)),
 in_per_rev(in_per_rev)
{
    this->MotorLF.setReversed(is_motor_reversed(MotorLFPort));
    this->MotorLB.setReversed(is_motor_reversed(MotorLBPort));
    this->MotorRF.setReversed(is_motor_reversed(MotorRFPort));
    this->MotorRB.setReversed(is_motor_reversed(MotorRBPort));
    this->fwdRotation.setReversed(is_motor_reversed(fwdRotatePort));
    this->sideRotation.setReversed(is_motor_reversed(sideRotatePort));

}

//sets the starting point and heading of the robot
void DriveBase::setStartingPoint(float startX, float startY, float startHeading ){
    this->x = startX;
    this->y = startY;
    this->setHeading(startHeading);
    this->heading = startHeading;
    this->resetFwdEncoder();
    //this->resetSideEncoder();
    this->fwdPosition = 0;
    this->sidePosition = 0;
}

//updates the position on the field of the robot
void DriveBase::updatePosition() {
    // getting current positions and current roatations and saving them into local variables
    float fwdPos = this->getFwdPosition();
    float sidePos = this->getSidePosition();
    float currentHead  = toolbox::fround(this->getHeading());

    
    //calculating deltas(difference between old and new positions)
    float deltaFwd = fwdPos - this->fwdPosition;
    float deltaSide = sidePos - this->sidePosition;
    float deltaHead = currentHead - this->heading;

    float deltaHeadRad = toolbox::degreesToRadians(deltaHead);

    //calculating distance between robot tracking center
    if (deltaHead==0) {
        localX = deltaSide;
        localY = deltaFwd;
    }
    else{
     localX = 2 * sin(deltaHeadRad/2) * ( (deltaSide/deltaHeadRad) + FWD_DISTANCE);
     localY = 2 * sin(deltaHeadRad/2) * ( (deltaFwd/deltaHeadRad) + SIDE_DISTANCE);
     }

    
    // converting to polar coordinates
    float vector_length;
    float angle_x_to_vector;
    if (localX == 0 && localY == 0) {
        vector_length = 0;
        angle_x_to_vector = 0;
    }
    else {
        vector_length = sqrt(pow(localX,2) + pow(localY, 2));
        angle_x_to_vector = atan2(localY, localX);
    }
    // calculate new global angle and convert back to cartesian: x = r cos θ , y = r sin θ
    float global_angle_polar_coordinates = toolbox::degreesToRadians(this->heading) - deltaHeadRad/2 - angle_x_to_vector;

    float deltaX = vector_length * cos(global_angle_polar_coordinates);
    float deltaY = vector_length * sin(global_angle_polar_coordinates);

    // updating x and y

   this->x += deltaX;
   this->y += deltaY;

    // updating stored positions and rotation
    this->fwdPosition = fwdPos;
    this->sidePosition = sidePos;
    this->heading = currentHead;
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

//returns inertial sensor heading in radians
double DriveBase::getHeadingRad(){
    double heading = this->gyroSensor.heading();
    return (heading * M_PI) / 180;
}

//returns inertial sensor rotation in radians
double DriveBase::getRotationRad(){
    double rotation = this->gyroSensor.rotation();
    return (rotation * M_PI) / 180;
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
     return toolbox::fround(this->fwdRotation.position(vex::rev)) * this->in_per_rev;
}

//returns current position of side tracking sensoir in inches
float DriveBase::getSidePosition(){
    return 0;
    // return toolbox::fround(this->sideRotation.position(vex::rev)) * this->in_per_rev;
}

//sets all motor brake-types to the type instucted
void DriveBase::SetBrake(vex::brakeType brake_type) {
    this->MotorLF.setBrake(brake_type);
    this->MotorLB.setBrake(brake_type);
    this->MotorRF.setBrake(brake_type);
    this->MotorRB.setBrake(brake_type);

}

//optimizes turning for PID so it turns to the left if the number is greater than 180
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




void DriveBase::DriveDistance(float distance){
    this->DriveDistance(distance, this->getHeading(), default_drive_Kp, default_drive_Ki, default_drive_Kd, default_drive_limit_integral, default_drive_exit_error, drive_default_min, drive_default_max, default_drive_timeout);
}

void DriveBase::DriveDistance(float distance, float heading){
    this->DriveDistance(distance, heading, default_drive_Kp, default_drive_Ki, default_drive_Kd, default_drive_limit_integral, default_drive_exit_error, drive_default_min, drive_default_max, default_drive_timeout);
}

void DriveBase::DriveDistance(float distance, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout){
    this->DriveDistance(distance, this->getHeading(), Kp, Ki, Kd, limit_integral, exit_error, drive_default_min, drive_default_max, timeout);
}

void DriveBase::DriveDistance(float distance, float heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, int timeout){
    this->DriveDistance(distance, heading, Kp, Ki, Kd, limit_integral, exit_error, drive_default_min, drive_default_max, timeout);
}

//drives forward or backward for the distance that is instructed
void DriveBase::DriveDistance(float distance, float dest_heading, float Kp, float Ki, float Kd, float limit_integral, float exit_error, float minOut, float maxOut, float timeout){
    PID pid = PID(Kp, Ki, Kd, limit_integral, exit_error, minOut, maxOut, timeout);
    pid.setDebug(true);
    //pid.setPIDmax(10);
    //pid.setPIDmin(0.1);
    PID heading_pid = PID(0.4, 0, 1, 0, 1, 15000);
    heading_pid.setPIDmax(6);
    heading_pid.setPIDmin(0);

    float destination = this->getFwdPosition() + distance;
    float error;
    float speed;
    float position;
    float heading_error;
    float heading_correction_speed;
    if (dest_heading < 0) { dest_heading += 360; };
    //std::cout << "#####################" << std::endl;

    do
    {   
        position = this->getFwdPosition();
        error = destination - position;
        std::cout << "Position:" << position << std::endl;
        speed = pid.calculate(error);

        heading_error = dest_heading - this->getHeading();
        heading_correction_speed = heading_pid.calculate(turnAngleOptimization(heading_error));
        this->RightMotors.spin(vex::fwd, speed - heading_correction_speed, vex::volt);
        this->LeftMotors.spin(vex::fwd, speed + heading_correction_speed, vex::volt);
        vex::wait(20, vex::msec);

    }while(!pid.isFinished());
    //std::cout<< "STOP" << std::endl;
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
    //std::cout << "STOP" << std::endl;

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
    //std::cout << "STOP" << std::endl;

    this->RightMotors.spin(vex::fwd, 0, vex::volt);
    this->LeftMotors.spin(vex::fwd, 0, vex::volt);
}

void DriveBase::turnToXY(float destX, float destY)
{
    float currHead = this->getHeading();
    float currX = this->getX();
    float currY = this->getY();
    destX -= currX;
    destY -= currY;
    this->TurnAngle(atan2(destX,destY) - currHead);
}   