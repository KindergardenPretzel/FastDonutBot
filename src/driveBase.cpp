#include "vex.h"
#include "driveBase.h"

bool DriveBase::is_motor_reversed(int motor){
    if(motor < 0)
    {
        return true;
    }
    return false;
}

DriveBase::DriveBase(int gyroPort, int fwdRotatePort, int sideRotatePort, int MotorLFPort, int MotorLBPort, int MotorRFPort, int MotorRBPort):
 fwdRotation(fwdRotatePort, true), 
 sideRotation(sideRotatePort, true), 
 gyroSensor(gyroPort),
 MotorLF(abs(MotorLFPort)),
 MotorLB(abs(MotorLBPort)),
 MotorRF(abs(MotorRFPort)),
 MotorRB(abs(MotorRBPort))
{
    MotorLF.setReversed(is_motor_reversed(MotorLFPort));
    MotorLB.setReversed(is_motor_reversed(MotorLBPort));
    MotorRF.setReversed(is_motor_reversed(MotorRFPort));
    MotorRB.setReversed(is_motor_reversed(MotorRBPort));
}