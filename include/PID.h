class PID
{
    private:
        float Kp;
        float Ki;
        float Kd;
        float error;
        float prevError;
        float firstRun;
        float limitIntegral;
        float integral;
        float minOutput;
        float maxOutput;
        float pidExitError;
        unsigned int timeout;
        unsigned int startTime;

    public:
        PID(float, float, float, float, float, int);
        float calculate(float);
        bool isFinished();
        void resetPID();
        void setPIDmax(float);
        void setPIDmin(float);
};