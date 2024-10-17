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


using namespace vex;

// A global instance of competition
competition Competition;
competition_debug Cdebug( Competition );

brain Brain;
controller Controller1 = controller(primary);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
motor intake = motor(PORT5,ratio18_1,false); 
motor scoring = motor(PORT6,ratio6_1,false);

float power_pct = 0.8;
// define your global instances of motors and other devices here


std::shared_ptr<DriveBase> robot(new DriveBase(PORT13, -PORT11, PORT12, -PORT1, -PORT2, PORT3, PORT4, 6.28));
//std::shared_ptr<Odometry> odom(new Odometry(robot));
Odometry odom = Odometry(robot);

//opens or closes clamp depending on whether the pneumatic cylynder is out or in
void clampFunc(){  
  if (!clamp.value()) {
        clamp.set(true);
      }
      else {
        clamp.set(false);
  };
}

//spinds the intake forward
void SpinnyThing(){
  intake.setVelocity(100.0, percent);
  intake.spin(forward);
  waitUntil((!Controller1.ButtonL1.pressing()));
  intake.stop();
}

//reverses the intake and conveyor belt
void reverseIntake(){
  intake.setVelocity(100.0, percent);
  intake.spin(reverse);
  scoring.setVelocity(50.0, percent);
  scoring.spin(reverse);
  waitUntil((!Controller1.ButtonR2.pressing()));
  intake.stop();
  scoring.stop();
}

//spins the intake and conveyo belt forward
void score(){
  static bool enabled = false;
  if (not enabled)
  {
  intake.setVelocity(100.0, percent);
  intake.spin(forward);
  scoring.setVelocity(50.0, percent);
  scoring.spin(forward);
  enabled = true;
  }
  else 
  {
  //waitUntil((!Controller1.ButtonR1.pressing()));
   scoring.stop();
   intake.stop();
   enabled = false;
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
  odom.setStartingPoint(10, 10, 90);
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

void autonomous(void) {
  //robot->SetBrake(brake);
  robot->DriveDistance(-30);
  wait(1,sec);
  //robot->DriveDistance(-10);
  //wait(1,sec);
  //robot->TurnAngle(340);
//wait(3,sec);
  //robot->TurnAngle(60);
//wait(3,sec);
 //robot->TurnAngle(120);
  //wait(1,sec);
  //robot->TurnAngle(90);

  //robot->TurnAngle(0);
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
    Controller1.ButtonR2.pressed(reverseIntake);
    //Controller1.ButtonL2.pressed(SpinnyThingBack);
    Controller1.ButtonR1.pressed(score);


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
