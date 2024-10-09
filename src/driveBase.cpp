#include "vex.h"
#include "drivebase.h"

bool DriveBase::is_motor_reversed(int motor){
    if(motor < 0)
    {
        return true;
    }
    return false;
}

DriveBase::DriveBase(int gyroPort, int fwdRotatePort, int sideRotatePort, 
                     int MotorLFPort, int MotorLBPort, int MotorRFPort, 
                     int MotorRBPort, float in_per_rev):
 fwdRotation(fwdRotatePort), 
 sideRotation(sideRotatePort),
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

double  DriveBase::getHeading(){
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
     return flround(this->fwdRotation.position(vex::rev)) * this->in_per_rev;
}

float DriveBase::getSidePosition(){
     return flround(this->sideRotation.position(vex::rev)) * this->in_per_rev;
}

void DriveBase::SetBrake(vex::brakeType brake_type) {
    this->MotorLF.setBrake(brake_type);
    this->MotorLB.setBrake(brake_type);
    this->MotorRF.setBrake(brake_type);
    this->MotorRB.setBrake(brake_type);

}