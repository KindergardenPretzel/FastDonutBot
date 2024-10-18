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
     return toolbox::fround(this->sideRotation.position(vex::rev)) * this->in_per_rev;
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
this->DriveDistance(distance, this->getHeading());
}

//drives forward or backward for the distance that is instructed
void DriveBase::DriveDistance(float distance, float dest_heading){
    PID pid = PID(0.8, 0, 0, .7, .3, 4000);
    pid.setPIDmax(10);
    pid.setPIDmin(0.1);
    PID heading_pid = PID(0.1, 0, 0, 2, 1, 15000);
    heading_pid.setPIDmax(6);
    heading_pid.setPIDmin(0.1);

    float destination = this->getFwdPosition() + distance;
    float error;
    float speed;
    float position;
    float heading_error;
    float heading_correction_speed;
    std::cout << "#####################" << std::endl;
    do
    {   
        position = this->getFwdPosition();
        error = destination - position;
        std::cout << "Position:" << position << std::endl;
        speed = pid.calculate(error);

        heading_error = dest_heading - this->getHeading();
        heading_correction_speed = heading_pid.calculate(turnAngleOptimization(heading_error));

        this->LeftMotors.spin(vex::fwd, speed + heading_correction_speed, vex::volt);
        this->RightMotors.spin(vex::fwd, speed - heading_correction_speed, vex::volt);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());
    std::cout<< "STOP" << std::endl;
    this->LeftMotors.stop(vex::brake);
    this->RightMotors.stop(vex::brake);
}

//turns to the angle instructed [0:359]
void DriveBase::TurnAngle(float angle)
{
    PID pid = PID(0.09, 0, 0, .7, 1, 2000);
    pid.setPIDmax(6);
    pid.setPIDmin(1.7);
    float error;
    float speed;
    float current_heading;
    std::cout<< "##########################" << std::endl<< std::endl<< std::endl<< std::endl;
    do{
        current_heading = this->getHeading();
        error = this->turnAngleOptimization(angle - current_heading);
        std::cout << "Heading:" << current_heading << std::endl;
        std::cout << "Error:" << error << std::endl;
        speed = pid.calculate(error);
        std::cout << "Speed:" << speed << std::endl;
        this->LeftMotors.spin(vex::fwd, speed, vex::volt);
        this->RightMotors.spin(vex::fwd, -speed, vex::volt);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());
    //std::cout << "STOP" << std::endl;

    this->LeftMotors.stop(vex::brake);
    this->RightMotors.stop(vex::brake);
}

void DriveBase::swingRight(float angle)
{
    PID pid = PID(0.4, 0, 0, .7, 1, 2000);
    pid.setPIDmax(6);
    pid.setPIDmin(1.7);
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

    this->LeftMotors.stop(vex::brake);
    this->RightMotors.stop(vex::brake);
}

void DriveBase::swingLeft(float angle)
{
    PID pid = PID(0.4, 0, 0, .7, 1, 2000);
    pid.setPIDmax(6);
    pid.setPIDmin(1.7);
    float error;
    float speed;
    float current_heading;
    do{
        current_heading = this->getHeading();
        error = this->turnAngleOptimization(angle - current_heading);
        speed = pid.calculate(error);
        this->RightMotors.spin(vex::fwd, speed, vex::volt);
        this->LeftMotors.stop(vex::hold);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());
    //std::cout << "STOP" << std::endl;

    this->LeftMotors.stop(vex::brake);
    this->RightMotors.stop(vex::brake);
}