#include "PID.h"
#include "toolbox.h"
#include <cmath>
#include <iostream>

PID::PID(float Ki, float Kp, float Kd, float limitIntegral, float pidExitError, int timeout): 
 Ki(Ki), 
 Kp(Kp), 
 Kd(Kd),
 limitIntegral(limitIntegral),
 pidExitError(pidExitError),
 timeout(timeout)
{
    this->firstRun = true;
}

void PID::resetPID(){ 
    this->prevError = 0;
    this->firstRun = true;
}

void PID::setPIDmax(float maxOutput) {
    this->maxOutput = maxOutput;
}

void PID::setPIDmin(float minOutput){
    this->minOutput = minOutput;
}

float PID::calculate(float error)
{
    float totalGain = 0;
    float proportionalGain = 0;
    float integralGain = 0;
    float derivativeGain = 0;

    this->error = error;
    proportionalGain = this->Kp * this->error;
    
    if (this->firstRun) {
       this->startTime = toolbox::highResTimerMs();
       this->prevError = this->error;
       this->firstRun = false;
    }

    if(fabs(this->error) < this->limitIntegral){
        this->integral += this->error;
    }
    else{
        this->integral = 0;
    }
    integralGain = this->Ki * this->integral;
    
    derivativeGain = (this->error - this->prevError) * this->Kd;
    this->prevError = this->error;

    totalGain = proportionalGain + integralGain + derivativeGain;
    
    // clamping gain
    if (std::abs(totalGain) > this->maxOutput) {
        totalGain = this->maxOutput * (std::abs(this->maxOutput) / this->maxOutput);
    }
    if (std::abs(totalGain) < this->minOutput) {
        totalGain = this->minOutput * (std::abs(this->minOutput) / this->minOutput);
    }    
    return totalGain;

}

bool PID::isFinished(){
    unsigned int runningTime = toolbox::highResTimerMs() - this->startTime;
    //std::cout << runningTime;
    if(fabs(this->error) < this->pidExitError)
    {
        return true;
    }
    if(runningTime > this->timeout)
    {
        return true;
    }
    return false;
}