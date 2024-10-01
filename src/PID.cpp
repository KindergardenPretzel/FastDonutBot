#include "PID.h"
#include <cmath>

PID::PID(float Ki, float Kp, float Kd, float limitIntegral): 
 Ki(Ki), 
 Kp(Kp), 
 Kd(Kd),
 limitIntegral(limitIntegral)
{
}

float PID::calculate(float current_position, float destination)//detination😊
{
    float totalGain = 0;
    float proportionalGain = 0;
    float integralGain = 0;
    float derivativeGain = 0;

    this->error = destination - current_position;
    proportionalGain = this->Kp * this->error;
    
    if (this->firstRun) {
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
    
    if (totalGain > this->maxOutput) {
        totalGain = this->maxOutput;
    }
    if (totalGain < this->minOutput) {
        totalGain = this->minOutput;
    }    
    return totalGain;

}