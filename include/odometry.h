#include "vex.h"

class Odometry
{
    private:
        vex::inertial gyroSensor;
        vex::rotation fwdRotation;
        vex::rotation sideRotation;
        float in_per_rev;
        const float SIDE_DISTANCE = 0.6;
        const float FWD_DISTANCE = 3;
    public:
        Odometry(int, int, int, float);
        void calibrateInertial();
        double  getRotation();
        void setRotation(double value);
        double  getHeading();
        void setHeading(double value);
        double getHeadingRad();
        double getRotationRad();
        void setStartingPoint(float, float, float);
        float degreesToRadians(float degrees);
        void updatePosition();

        float heading;
        float x;
        float y;
        float fwdPosition;
        float sidePosition;
        float localY;
        float localX;
};