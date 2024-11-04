#include "vex.h"
#include "drivebase.h"
#include <memory>

class Odometry
{
    private:
        //vex::inertial gyroSensor;
        //vex::rotation fwdRotation;
        //vex::rotation sideRotation;
        const float SIDE_DISTANCE = 0.6;
        const float FWD_DISTANCE = 3;
        std::shared_ptr<DriveBase> chassis;

    public:
        Odometry(std::shared_ptr<DriveBase>& chassis);
        //void calibrateInertial();
        /*double  getRotation();
        void setRotation(double value);
        double  getHeading();
        void setHeading(double value);
        double getHeadingRad();
        double getRotationRad(); */
        void setStartingPoint(float, float, float);
        //float degreesToRadians(float degrees);
        void updatePosition();

        float heading;
        float x;
        float y;
        float fwdPosition;
        float sidePosition;
        float localY;
        float localX;
};