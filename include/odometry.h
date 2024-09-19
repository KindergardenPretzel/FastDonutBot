#include "vex.h"

class Odometry
{
    private:
        vex::inertial gyroSensor;
        vex::rotation fwdRotation;
        vex::rotation sideRotation;
        double heading;
        double x;
        double y;
        float fwdPosition;
        float sidePosition;
        const float wheelDiameter = 2.75;
    public:
        Odometry(int, int, int);
        void calibrateInertial();
        double  getRotation();
        void setRotation(double value);
        double  getHeading();
        void setHeading(double value);
        double getHeadingRad();
        double getRotationRad();
        void setStartingPoint(double x, double y);
};