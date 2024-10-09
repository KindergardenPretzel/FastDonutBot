#include "odometry.h"
#include "toolbox.h"


Odometry::Odometry(std::shared_ptr<DriveBase>& chassis):
 chassis(chassis)
{
    //std::shared_ptr<DriveBase> ptr(robot)
};
/*
void Odometry::calibrateInertial() {
    chassis->calibrateInertial;
}

double  Odometry::getRotation(){
    return this->gyroSensor.rotation();
}

void Odometry::setRotation(double value){
    this->gyroSensor.setRotation(value, vex::deg);
}

double  Odometry::getHeading(){
    return this->gyroSensor.heading();
}

void Odometry::setHeading(double value){
    this->gyroSensor.setHeading(value, vex::deg);
}

double Odometry::getHeadingRad(){
    double heading = this->gyroSensor.heading();
    return (heading * M_PI) / 180;
}

double Odometry::getRotationRad(){
    double rotation = this->gyroSensor.rotation();
    return (rotation * M_PI) / 180;
}
*/
void Odometry::setStartingPoint(float x, float y, float heading ){
    this->x = x;
    this->y = y;
    chassis->setHeading(heading);
    this->heading = heading;
    chassis->resetFwdEncoder();
    chassis->resetSideEncoder();
    this->fwdPosition = 0;
    this->sidePosition = 0;
}

float Odometry::degreesToRadians(float degrees){
    return (degrees * M_PI) / 180;
}

void Odometry::updatePosition() {
    // getting current positions and current roatations and saving them into local variables
    float fwdPos = chassis->getFwdPosition();
    float sidePos = chassis->getSidePosition();
    float currentHead  = toolbox::fround(chassis->getHeading());

    
    //calculating deltas
    float deltaFwd = fwdPos - this->fwdPosition;
    float deltaSide = sidePos - this->sidePosition;
    float deltaHead = currentHead - this->heading;

    float deltaHeadRad = this->degreesToRadians(deltaHead);
    //calculating robot center path
    if (deltaHead==0) {
        localX = deltaSide;
        localY = deltaFwd;
    }
    else{
     localX = 2 * sin(deltaHeadRad/2) * ( (deltaSide/deltaHeadRad) + FWD_DISTANCE);
     localY = 2 * sin(deltaHeadRad/2) * ( (deltaFwd/deltaHeadRad) + SIDE_DISTANCE);
     }

    
    // converting to polar coordinates
    float vector_length = sqrt(pow(localX,2) + pow(localY, 2));
    float angle_x_to_vector = atan2(localY, localX);

    // calculate new global angle and convert back to cartesian: x = r cos θ , y = r sin θ
    float global_angle_polar_coordinates = this->degreesToRadians(this->heading) - deltaHeadRad/2 - angle_x_to_vector;

    float deltaX = vector_length * cos(global_angle_polar_coordinates);
    float deltaY = vector_length * sin(global_angle_polar_coordinates);

    // updating x and y

   this->x += deltaX;
   this->y += deltaY;

    // updating stored positions and rotation
    this->fwdPosition = fwdPos;
    this->sidePosition = sidePos;
    this->heading = currentHead;
}