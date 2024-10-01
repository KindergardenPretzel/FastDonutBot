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

    public:
        PID(float, float, float, float);
        float calculate(float, float);
};