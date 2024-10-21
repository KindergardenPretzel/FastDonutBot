class PID
{
    private:
        float Kp;
        float Ki;
        float Kd;
        float error;
        float prevError;
        bool firstRun{true};
        float limitIntegral;
        float integral;
        float minOutput{1};
        float maxOutput{12};
        float pidExitError;
        unsigned int timeout;
        unsigned int startTime;

    public:
        PID(float Kp, float Ki, float Kd, float limitIntegral, float pidExitError, int timeout);
        PID(float Kp, float Ki, float Kd, float limitIntegral, float pidExitError, float minOutput, float maxOutput, int timeout); 
        float calculate(float);
        bool isFinished();
        void resetPID();
        void setPIDmax(float);
        void setPIDmin(float);
};