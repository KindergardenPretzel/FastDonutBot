#include "PID.h"
#include "toolbox.h"
#include <cmath>
#include <iostream>

//PID class constructor
PID::PID(float Kp, float Ki, float Kd, float limitIntegral, float pidExitError, int timeout): 
 Ki(Ki), 
 Kp(Kp), 
 Kd(Kd),
 limitIntegral(limitIntegral),
 pidExitError(pidExitError),
 timeout(timeout)
{
    this->firstRun = true;
}

//resets the previous error and sets first run to true
void PID::resetPID(){ 
    this->prevError = 0;
    this->firstRun = true;
}

//sets maxOutput to the given maxOutput
void PID::setPIDmax(float maxOutput) {
    this->maxOutput = maxOutput;
}

//sets minOutput to the given minOutput
void PID::setPIDmin(float minOutput){
    this->minOutput = minOutput;
}

//calculates all PID
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
        totalGain = this->maxOutput * (std::abs(totalGain) / totalGain);
    }
    if (std::abs(totalGain) < this->minOutput) {
        totalGain = this->minOutput * (std::abs(totalGain) / totalGain);
    }    
    std::cout << "P:" << proportionalGain << std::endl;
    std::cout << "I:" << integralGain << std::endl;
    std::cout << "D:" << derivativeGain << std::endl;
    std::cout << "Speed:" << totalGain << std::endl;
    std::cout << "Error:" << error << std::endl;
    return totalGain;

}

//checks if the PID drive has finished
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