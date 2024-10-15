#include "vex.h"
#include "drivebase.h"
#include "PID.h"
#include <iostream>

bool DriveBase::is_motor_reversed(int motor){
    if(motor - 1  < 0)
    {
        return true;
    }
    return false;
}

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

void DriveBase::calibrateInertial() {
    this->gyroSensor.calibrate();
    while (this->gyroSensor.isCalibrating())
    {
        vex::wait(20, vex::msec);
    };
}

double  DriveBase::getRotation(){
    return this->gyroSensor.rotation();
}

void DriveBase::setRotation(double value){
    this->gyroSensor.setRotation(value, vex::deg);
}

float DriveBase::getHeading(){
    return this->gyroSensor.heading();
}

void DriveBase::setHeading(double value){
    this->gyroSensor.setHeading(value, vex::deg);
}

double DriveBase::getHeadingRad(){
    double heading = this->gyroSensor.heading();
    return (heading * M_PI) / 180;
}

double DriveBase::getRotationRad(){
    double rotation = this->gyroSensor.rotation();
    return (rotation * M_PI) / 180;
}

void DriveBase::resetFwdEncoder(){
     this->fwdRotation.resetPosition();
}

void DriveBase::resetSideEncoder(){
     this->sideRotation.resetPosition();
}

float DriveBase::getFwdPosition(){
     return toolbox::fround(this->fwdRotation.position(vex::rev)) * this->in_per_rev;
}

float DriveBase::getSidePosition(){
     return toolbox::fround(this->sideRotation.position(vex::rev)) * this->in_per_rev;
}

void DriveBase::SetBrake(vex::brakeType brake_type) {
    this->MotorLF.setBrake(brake_type);
    this->MotorLB.setBrake(brake_type);
    this->MotorRF.setBrake(brake_type);
    this->MotorRB.setBrake(brake_type);

}

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

void DriveBase::FwdDriveDistance(float distance){
    PID pid = PID(1.5, 0, 0, .7, .5, 3000);
    pid.setPIDmax(8);
    pid.setPIDmin(3.8);
    float destination = this->getFwdPosition() + distance;
    float error;
    float speed;
    do
    {
        error = destination - this->getFwdPosition();
        speed = pid.calculate(error);
        this->LeftMotors.spin(vex::fwd, speed, vex::volt);
        this->RightMotors.spin(vex::fwd, speed, vex::volt);
        vex::wait(10, vex::msec);

    }while(!pid.isFinished());

    this->LeftMotors.stop(vex::brake);
    this->RightMotors.stop(vex::brake);
}

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