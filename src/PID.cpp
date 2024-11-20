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


PID::PID(float Kp, float Ki, float Kd, float limitIntegral, float pidExitError, float minOutput, float maxOutput, int timeout): 
 Ki(Ki), 
 Kp(Kp), 
 Kd(Kd),
 limitIntegral(limitIntegral),
 pidExitError(pidExitError),
 minOutput(minOutput),
 maxOutput(maxOutput),
 timeout(timeout)
{
    this->firstRun = true;
}


//enable Debug Output to console.
void PID::setDebug(bool enabled) {
    this->debugOn = true;
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
    //derivativeGain = (this->prevError - this->error) * this->Kd;
    

    totalGain = proportionalGain + integralGain + derivativeGain;
    
    // clamping gain
    if (std::abs(totalGain) > this->maxOutput) {
        totalGain = this->maxOutput * (std::abs(proportionalGain) / proportionalGain);
    }
    if (std::abs(totalGain) < this->minOutput) {
        totalGain = this->minOutput * (std::abs(proportionalGain) / proportionalGain);
    }  
    /*if(debugOn)  
    {
        std::cout << "===============" << std::endl;   
        std::cout << "P:" << proportionalGain << std::endl;
        std::cout << "I:" << integralGain << std::endl;
        std::cout << "D:" << derivativeGain << std::endl;
        std::cout << "Speed:" << totalGain << std::endl;
        std::cout << "Error:" << error << std::endl;
        std::cout << "Prev Error:" << this->prevError << std::endl;
        std::cout << "Delta Error:" << this->error - this->prevError << std::endl;
    };*/
    
    this->prevError = this->error;
    return totalGain;

}

//checks if the PID drive has finished
bool PID::isFinished(){
    unsigned int runningTime = toolbox::highResTimerMs() - this->startTime;
    if(fabs(this->error) < this->pidExitError)
    {
        if(debugOn)  
        {
            std::cout << "PID Eror Exit. Running Time:" << runningTime << std::endl;
        };
        return true;
    }
    if(runningTime > this->timeout)
    {
        if(debugOn)  
        {
            std::cout << "PID Timeout Exit. Running Time:" << runningTime << std::endl;
        };
        return true;
    }
    return false;
}