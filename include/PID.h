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
        PID(float, float, float, float, float, int);
        float calculate(float);
        bool isFinished();
        void resetPID();
        void setPIDmax(float);
        void setPIDmin(float);
};