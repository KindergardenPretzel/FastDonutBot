/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Oleg Khavroniuk                                           */
/*    Created:      5/10/2024, 4:14:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "odometry.h"
//#include "drivebase.h"
#include "comp_debug.h"
#include "PID.h"
//#include "toolbox.h"
#include <memory>
#include <iostream>


using namespace vex;

// A global instance of competition
competition Competition;
//competition_debug Cdebug( Competition );

brain Brain;
controller Controller1 = controller(primary);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
digital_out intake_lift = digital_out(Brain.ThreeWirePort.B);

motor intake = motor(PORT5,ratio6_1,false); 
motor scoring = motor(PORT6,ratio6_1,false);

optical eyeball = optical(PORT9);
distance DistanceSensor = distance(PORT7);


float power_pct = 0.8;
// define your global instances of motors and other devices here


std::shared_ptr<DriveBase> robot(new DriveBase(PORT13, -PORT11, PORT12, -PORT1, -PORT2, PORT3, PORT4, 6.28));
//std::shared_ptr<Odometry> odom(new Odometry(robot));
Odometry odom = Odometry(robot);
bool isBeltSpinning = false;
bool isStopperEnabled = false;

void score();

/*enum Colors {
  OWN = 16711680, // RED 
  OPPOSITE = 255 // BLUE
};*/
enum Colors {
  OWN = 255, // BLUE 
  OPPOSITE = 16711680 // RED
};

//opens or closes clamp depending on whether the pneumatic cylynder is out or in
void clampFunc(){  
  if (!clamp.value()) {
        clamp.set(true);
      }
      else {
        clamp.set(false);
  };
}

//lifts intake using pneumatic cylynder
void lift_intake(){
  if (!intake_lift.value()) {
        intake_lift.set(true);
      }
      else {
        intake_lift.set(false);
  };
}

//spins the intake forward
void intake_spin_fwd(int speed=90){
  intake.spin(vex::fwd, speed, vex::pct);
}

//spins the intake reverse
void intake_spin_back(int speed=90){
  intake.spin(vex::fwd, -speed, vex::pct);
}

//spins the intake forward
void intake_stop(){
  intake.stop();
}


//reverses the intake and conveyor belt
void reverseIntake(){
  intake_spin_back(70);
  scoring.setVelocity(50.0, percent);
  scoring.spin(reverse);
  waitUntil((!Controller1.ButtonR2.pressing()));
  intake_stop();
  scoring.stop();
}

int stopWhenColorSeenTask()
{
  eyeball.setLightPower(75, vex::pct);
  eyeball.setLight(ledState::on);
  bool toggle = true;
  while (toggle) {
    color detectColor = eyeball.color();
    //std::cout << detectColor << std::endl;
    //vex::wait(10, msec);
    if (eyeball.color() == OWN) {
      toggle = false;
      score();
      eyeball.setLight(ledState::off);
      isStopperEnabled = false;
    }
   vex::wait(10, msec);
  }
  return 0;
}

void stopWhenColorSeen()
{
  if (!isStopperEnabled) {
    vex::task Stop(stopWhenColorSeenTask);
    isStopperEnabled = true; 
  }
  else{
    task::stop(stopWhenColorSeenTask);
    eyeball.setLight(ledState::off);
    isStopperEnabled = false; 
  }

}

// checks if intake and belt motor spinning
//bool isBeltSpinning()
//{
 // if(scoring.power() > 0 || intake.power() > 0)
 // {
 //   return true;
 // }
 // return false;
//}


//spins the intake and conveyo belt forward
void score(){
  //vex::task Color(stopWhenColorSeen);

  //static bool enabled = false;
  if (!isBeltSpinning)
  {
  //intake.setVelocity(90.0, percent);
  //intake.spin(forward);

  intake_spin_fwd(90);
  scoring.setVelocity(65.0, percent);
  scoring.spin(forward);
  isBeltSpinning = true;
  }
  else 
  {
   scoring.stop();
   //intake.stop();
   intake_stop();
   isBeltSpinning = false;
  };
}

//task that updates the robots position
int updatePos()
{
    while(true)
    {
        odom.updatePosition();
        this_thread::sleep_for(10);
    }
    return(0);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//prints info on the brain screen
int ShowMeInfo(){
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor(red);

  while(true) {
  Brain.Screen.setCursor(4,2);
  Brain.Screen.print("X: %f, Y: %f", odom.x, odom.y);
  Brain.Screen.setCursor(5,2);
  Brain.Screen.print("Heading: %f", robot->getHeading());

  Brain.Screen.setCursor(6,2);
  Brain.Screen.print("position: %f", robot->getFwdPosition());
  //Controller1.Screen.print("X: %f, Y: %f", OdometryObjPtr->X, OdometryObjPtr->Y);

  this_thread::sleep_for(40);
  Brain.Screen.clearScreen();
  };
  return 0;
}

void pre_auton(void) {
  Brain.Screen.setCursor(4,3);
  Brain.Screen.print("Calibrating Inertial Sensor");
  robot->calibrateInertial();
  Brain.Screen.clearScreen();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auton_red_two_stakes() {
  
/* exmaple 
  robot->DriveDistance(-20, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  robot->DriveDistance(5, 1.2, 0.07, 0, 1.5, 0.5, 5000);
  robot->DriveDistance(-10, 1, 0.07, 0, 0.5, 0.5, 5000);
*/
// angle, P,I,D,startIntegral, exit, min, max, timeout
//robot->TurnAngle(175, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);

  odom.setStartingPoint(10, 10, 90);

  robot->DriveDistance(-36, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  robot->swingLeftHold(120);
  robot->DriveDistance(-8, 1, 0.07, 0, 0.5, 0.5, 2000);
  robot->DriveDistance(-4.5, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(30,msec);
  clampFunc();
  wait(20,msec);
  score();
  robot->swingLeftHold(70);
  robot->DriveDistance(12, 1, 0.07, 0, 0.5, 0.5, 2000);
  wait(400,msec);
  //intake_stop();
  //clampFunc();
}

void auton_blue_two_stakes() {
  odom.setStartingPoint(10, 10, 90);

  robot->DriveDistance(-36, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  robot->swingRightHold(60);
  robot->DriveDistance(-8, 1, 0.07, 0, 0.5, 0.5, 2000);
  robot->DriveDistance(-4.5, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(30,msec);
  clampFunc();
  wait(20,msec);
  score();
  robot->swingRightHold(110);
  robot->DriveDistance(12, 1, 0.07, 0, 0.5, 0.5, 2000);
  wait(400,msec);
  //intake_stop();
  //clampFunc();
}

void auton_red_left() {
  odom.setStartingPoint(10, 10, 2);
  robot->DriveDistance(5, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(304, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  lift_intake();
  robot->DriveDistance(4, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  intake_spin_fwd();
  wait(20,msec);
  lift_intake();
  wait(20,msec);
  robot->DriveDistance(7, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(273, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - 5), 1.1, 0.07, 0, 1.5, 0.5, 2000);
  intake_stop();
  wait(20,msec);
  score();
  wait(1600,msec);
  robot->DriveDistance(10, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(30, msec);
  score();
  wait(20,msec);
  robot->TurnAngle(30, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(300,msec);
  robot->DriveDistance(-30, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  clampFunc();
  wait(30,msec);
  robot->TurnAngle(200, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(30, msec);
  score();
  wait(200, msec);
  robot->DriveDistance(24, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20, msec);
  robot->DriveDistance(-20, 300, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(20, msec);
  robot->DriveDistance(15, 1.2, 0.07, 0, 1.5, 0.5, 2000);
}

void auton_blue_left() {
  odom.setStartingPoint(10, 10, 2);
  robot->DriveDistance(5, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(304, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  lift_intake();
  robot->DriveDistance(4, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  intake_spin_fwd();
  wait(20,msec);
  lift_intake();
  wait(20,msec);
  robot->DriveDistance(7, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(273, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - 5.5), 1.1, 0.07, 0, 1.5, 0.5, 2000);
  intake_stop();
  wait(20,msec);
  score();
  wait(1600,msec);
  robot->DriveDistance(10, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(30, msec);
  score();
  wait(20,msec);
  robot->TurnAngle(30, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(300,msec);
  robot->DriveDistance(-30, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  clampFunc();
  wait(30,msec);
  robot->TurnAngle(165, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(30, msec);
  score();
  wait(200, msec);
  robot->DriveDistance(26, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(30,msec);
  robot->TurnAngle(300, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(100, msec);
  robot->DriveDistance(24, 0.5, 0.07, 0, 1.5, 0.5, 2000);
  wait(100, msec);
  clampFunc();
}

void auton_red_right() {
  odom.setStartingPoint(10, 10, 180);
  lift_intake();
  robot->DriveDistance(5, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(20,msec);
  robot->TurnAngle(239, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  robot->DriveDistance(4, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(20,msec);
  intake_spin_fwd();
  wait(20,msec);
  lift_intake();
  wait(20,msec);
  robot->DriveDistance(7, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(270, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - 5.5), 1.1, 0.07, 0, 1.5, 0.5, 2000);
  intake_stop();
  wait(20,msec);
  score();
  wait(1600,msec);
  robot->DriveDistance(10, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(30, msec);
  score();
  wait(20,msec);
  robot->TurnAngle(153, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);//127
  wait(300,msec);
  robot->DriveDistance(-30, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  clampFunc();
  wait(30,msec);
  robot->TurnAngle(10, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(30, msec);
  score();
  wait(400, msec);
  robot->DriveDistance(26, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(30,msec);
  robot->TurnAngle(230, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(100, msec);
  robot->DriveDistance(24, 0.5, 0.07, 0, 1.5, 0.5, 2000);
  wait(100, msec);
  clampFunc();
}


void auton_blue_right() {
  odom.setStartingPoint(10, 10, 180);
  lift_intake();
  robot->DriveDistance(5, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(20,msec);
  robot->TurnAngle(239, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  robot->DriveDistance(4, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(20,msec);
  intake_spin_fwd();
  wait(20,msec);
  lift_intake();
  wait(20,msec);
  robot->DriveDistance(7, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  robot->TurnAngle(270, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(20,msec);
  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - 5.5), 1.1, 0.07, 0, 1.5, 0.5, 2000);
  intake_stop();
  wait(20,msec);
  score();
  wait(1600,msec);
  robot->DriveDistance(10, 1.2, 0.07, 0, 1.5, 0.5, 2000);
  wait(30, msec);
  score();
  wait(20,msec);
  robot->TurnAngle(153, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);//127
  wait(300,msec);
  robot->DriveDistance(-30, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(20,msec);
  clampFunc();
  wait(30,msec);
  robot->TurnAngle(340, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(30, msec);
  score();
  wait(400, msec);
  robot->DriveDistance(24, 0.5, 0.07, 0, 2.0, 0.5, 5000);
  wait(30,msec);
  robot->TurnAngle(205, 0.15, 0.01, 0, 15, 2, 2, 11, 2000);
  wait(100, msec);
  robot->DriveDistance(24, 0.5, 0.07, 0, 1.5, 0.5, 2000);
  wait(100, msec);
  clampFunc();
}

void autonomous(void) {
//auton_red_left();
//auton_blue_left();
//auton_red_right();
auton_blue_right();

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  robot->SetBrake(coast);

  while (1) {
    float throttle = Controller1.Axis3.position();
    float turn = Controller1.Axis1.position();

    robot->LeftMotors.spin(vex::fwd, power_pct * 0.0001 * pow(throttle+turn, 3), vex::pct);
    robot->RightMotors.spin(vex::fwd, power_pct * 0.0001 * pow(throttle-turn, 3), vex::pct);

    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
      //Brain.Screen.setCursor(4,2);
      //Brain.Screen.print("throttle: %f, turn: %f", throttle, turn);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

    //starts all functions
    Controller1.ButtonL1.pressed(clampFunc);
    Controller1.ButtonL2.pressed(lift_intake);
    Controller1.ButtonR2.pressed(reverseIntake);
    Controller1.ButtonR1.pressed(score);
    Controller1.ButtonY.pressed(stopWhenColorSeen);

  // Run the pre-autonomous function.
  pre_auton();

  
  // start debug output;
  vex::task Debug(ShowMeInfo);
  vex::task Position(updatePos);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  //autonomous();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
