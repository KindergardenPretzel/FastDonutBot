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
#include "comp_debug.h"

#include <memory>

std::shared_ptr<Odometry> odom(new Odometry(vex::PORT11, vex::PORT12, vex::PORT13, 6.28));


using namespace vex;

// A global instance of competition
competition Competition;
competition_debug Cdebug( Competition );

brain Brain;
controller Controller1 = controller(primary);
motor MotorLF = motor(PORT1, ratio6_1, true);
motor MotorLB = motor(PORT2, ratio6_1, true);
motor MotorRF = motor(PORT3, ratio6_1, false); 
motor MotorRB = motor(PORT4, ratio6_1, false);
motor intake = motor(PORT5,ratio18_1,true); 
motor scoring = motor(PORT6,ratio6_1,true);
motor_group LeftMotors = motor_group(MotorLF, MotorLB);
motor_group RightMotors = motor_group(MotorRF, MotorRB);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);

float power_pct = 0.8;
// define your global instances of motors and other devices here
void clampFunc(){  
  if (!clamp.value()) {
        clamp.set(true);
      }
      else {
        clamp.set(false);
  };
}

void SpinnyThing(){
  intake.setVelocity(100.0, percent);
  intake.spin(forward);
  waitUntil((!Controller1.ButtonL1.pressing()));
  intake.stop();
}

void reverseIntake(){
  intake.setVelocity(100.0, percent);
  intake.spin(reverse);
  scoring.setVelocity(50.0, percent);
  scoring.spin(reverse);
  waitUntil((!Controller1.ButtonR2.pressing()));
  intake.stop();
  scoring.stop();
}

void score(){
  intake.setVelocity(100.0, percent);
  intake.spin(forward);
  scoring.setVelocity(50.0, percent);
  scoring.spin(forward);
  waitUntil((!Controller1.ButtonR1.pressing()));
  scoring.stop();
  intake.stop();
}

int updatePos()
{
    while(true)
    {
        odom->updatePosition();
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
int ShowMeInfo(){
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor(red);
  while(true) {

  Brain.Screen.setCursor(6,2);
  Brain.Screen.print("X: %f, Y: %f", odom->x,odom->y);
  Brain.Screen.setCursor(7,2);
  Brain.Screen.print("Heading: %f", odom->getHeading());

  Brain.Screen.setCursor(8,2);
  Brain.Screen.print("LX: %f, LY: %f", odom->localX, odom->localY);
  //Controller1.Screen.print("X: %f, Y: %f", OdometryObjPtr->X, OdometryObjPtr->Y);

  this_thread::sleep_for(40);
  Brain.Screen.clearScreen();
  };
  return 0;
}

void pre_auton(void) {
  Brain.Screen.setCursor(4,3);
  Brain.Screen.print("Calibrating Inertial Sensor");
  odom->calibrateInertial();
  Brain.Screen.clearScreen();
  odom->setStartingPoint(10, 10, 0);
  odom->setHeading(90);
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
  // User control code here, inside the loop
  MotorRF.setBrake(brake);
  MotorRB.setBrake(brake);
  MotorLF.setBrake(brake);
  MotorLB.setBrake(brake);
  while (1) {
    float throttle = Controller1.Axis3.position();
    float turn = Controller1.Axis1.position();

      //Brain.Screen.setCursor(4,2);
      //Brain.Screen.print("throttle: %f, turn: %f", throttle, turn);


    LeftMotors.spin(vex::fwd, power_pct * 0.0001 * pow(throttle+turn, 3), vex::pct);
    RightMotors.spin(vex::fwd, power_pct * 0.0001 * pow(throttle-turn, 3), vex::pct);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    /*RightMotors.setVelocity((Controller1.Axis3.position() - Controller1.Axis1.position() * .5), percent);
    LeftMotors.setVelocity((Controller1.Axis1.position() * .5 + Controller1.Axis3.position()), percent);
    RightMotors.spin(forward);
    LeftMotors.spin(forward);*/
    //x = (Controller1.Axis3.position() - 0.65 * Controller1.Axis1.position());
    //y = (0.65 * Controller1.Axis1.position() + Controller1.Axis3.position());
    //RightMotors.spin(fwd, 0.01 * x * fabs(x), pct);
    //LeftMotors.spin(fwd, 0.01 * y * fabs(y), pct);
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
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



  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
