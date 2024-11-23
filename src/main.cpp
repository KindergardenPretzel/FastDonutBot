/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Oleg Khavroniuk                                           */
/*    Created:      5/10/2024, 4:14:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "drivebase.h"
#include "comp_debug.h"
#include "PID.h"
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
digital_out bypass = digital_out(Brain.ThreeWirePort.H);
digital_out highStakeLift = digital_out(Brain.ThreeWirePort.G);
limit auton_switch = limit(Brain.ThreeWirePort.E);

motor intake = motor(PORT5,ratio6_1,false); 
motor scoring = motor(PORT6,ratio6_1,true);

optical eyeball = optical(PORT14);
distance DistanceSensor = distance(PORT7);


float power_pct = 0.8;
// define your global instances of motors and other devices here

// Inertial, ForwardTrackingWheel, SideTrackingWheel, LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor, trackingIntchesPerRevolution
std::shared_ptr<DriveBase> robot(new DriveBase(PORT13, -PORT11, PORT12, -PORT1, -PORT2, PORT3, PORT4, 6.28));

bool isBeltSpinning = false;
bool isStopperEnabled = false;
bool autonEnabled = false;
int autonId = 1;
bool isBypassEnabled = false;

float redStakeApproachDist = 5.2;
float blueStakeApproachDist = 5.2;

void score();

enum Alliance {
  RED = 16711680,
  BLUE = 255
};

Alliance OWN;
Alliance OPPOSITE;
/*enum Colors {
  OWN = 255, // BLUE 
  OPPOSITE = 16711680 // RED
};*/

//opens or closes clamp depending on whether the pneumatic cylynder is out or in
void clampFunc(){  
  if (!clamp.value()) {
        clamp.set(true);
      }
      else {
        clamp.set(false);
  };
}

void setAlliance(Alliance selector)//Allience
{
  if(selector == RED)
  {
    OWN = RED;
    OPPOSITE = BLUE;
  }
  else if(selector == BLUE)
  {
    OWN = BLUE;
    OPPOSITE = RED;
  }
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

/*
void liftRamp()
{
  if(highStakeLift.value())
  {
    highStakeLift.set(false);
  }
  else if(!highStakeLift.value())
  {
    if (isBeltSpinning) {
      scoring.stop();
    }
    highStakeLift.set(true);
  }
}
*/

int ColorSensing()
{
  eyeball.setLightPower(50, vex::pct);
  eyeball.setLight(ledState::on);
  while (true) {
    color detectColor = eyeball.color();
    //std::cout << detectColor << std::endl;
    //vex::wait(10, msec);
    if (eyeball.color() == OWN && isStopperEnabled && isBeltSpinning){
      wait(10, msec);
      score();
      isStopperEnabled = false;
    }
    else if(eyeball.color() == OPPOSITE && isBypassEnabled)
    {
      bypass.set(true);
    }
    else if(eyeball.color() == OWN && isBypassEnabled)
    {
      bypass.set(false);
    }
   vex::wait(10, msec);
  }
  return 0;
}

void stopWhenColorSeen()
{
  if (!isStopperEnabled) {
    isStopperEnabled = true; 
  }
  else{
    isStopperEnabled = false; 
  }

}

void enableBypass()
{
  if (!isBypassEnabled) {
    isBypassEnabled = true; 
  }
  else{
    isBypassEnabled = false;
    bypass.set(false);
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
  if (!isBeltSpinning)
 {
//  if (!highStakeLift.value()) {
    intake_spin_fwd(70);
    scoring.spin(forward, 75, vex::pct);
 //   }
 //   else
 //   {
//      scoring.spin(forward, -50, vex::pct);
//    }
    isBeltSpinning = true;
  }
  else 
  {
   scoring.stop();
   intake_stop();
   isBeltSpinning = false;
  };
}

//task that updates the robots position
int updatePos()
{
    while(true)
    {
        robot->updatePosition();
        this_thread::sleep_for(5);
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
  float heading_angle;
  while(true) {

    Brain.Screen.setCursor(3,2);
    Brain.Screen.print("X: %f, Y: %f", robot->getX(), robot->getY());

    Brain.Screen.setCursor(4,2);
    heading_angle = robot->getHeading();
    //if (heading_angle > 180) { heading_angle -= 360;};
    Brain.Screen.print("Heading: %f", heading_angle);

    Brain.Screen.setCursor(5,2);
    Brain.Screen.print("position: %f", robot->getFwdPosition());

    Brain.Screen.setCursor(6,2);
    Brain.Screen.print("X1: %f, Y1: %f", robot->x1, robot->y1);
  
  if (clamp.value()) 
  {
    Controller1.Screen.setCursor(3,5);
    Controller1.Screen.print("C");
  }

  if (isBypassEnabled) 
  {
    Controller1.Screen.setCursor(3,13);
    Controller1.Screen.print("B");
  }
  if (isStopperEnabled) 
  {
    Controller1.Screen.setCursor(3,10);
    Controller1.Screen.print("S");
  }

  vex::wait(200, msec);

  Brain.Screen.clearScreen();
  Controller1.Screen.clearLine(3);

  };
  return 0;
}

void auton_select() {
  autonId ++;
  if(autonId > 6)
    {
      autonId = 1;
    }
  Brain.Screen.clearLine(8);
  Controller1.Screen.clearLine(3);
}

void pre_auton(void) {
  // auton selector callbacks
  auton_switch.pressed(auton_select);
  Brain.Screen.pressed(auton_select);

  bypass.set(false);
  setAlliance(RED);
  highStakeLift.set(false);
  Brain.Screen.setCursor(4,3);
  Brain.Screen.print("Calibrating Inertial Sensor");
  robot->calibrateInertial();
  Brain.Screen.clearScreen();

  while(!autonEnabled)
  {
    switch(autonId)
    {
      case 1: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Red Left Side Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: RL");
      break;
      case 2: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Red Right Side Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: RR");
      break;
      case 3: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Blue Left Side Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: BL");
      break;
      case 4: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Blue Right Side Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: BR");
      break;
      case 5: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Test Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: TST");
      break;   
      case 6: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Skills Id: %d", autonId);
        Controller1.Screen.setCursor(3,10);
        Controller1.Screen.print("A: SKLS");
      break;   
    }

    /*if(Brain.Screen.pressing() || auton_switch.pressing())
    {
      while(Brain.Screen.pressing() || auton_switch.pressing())
      {}
      autonId ++;
      if(autonId > 6)
      {
        autonId = 1;
      }
      wait(10, msec);
      Brain.Screen.clearLine(8);
      Controller1.Screen.clearLine(3);
    }*/

      //wait(1000, msec);

  }

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





void auton_blue_left() {

 robot->default_drive_exit_error = 1;
  robot->default_drive_max = 6;
  robot->default_heading_max = 10;


  // take middle ring
  enableBypass(); 
  lift_intake();
  robot->DriveDistance(9, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(20,msec);
  lift_intake();
  intake_spin_fwd();
  wait(400,msec);
  intake_stop();
  robot->DriveDistance(12, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(40,msec);
  robot->TurnAngle(90);


  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - redStakeApproachDist));
  wait(20,msec);
  // score two rings
  score();
  wait(600,msec);
  score();
  wait(20, msec);
  // drive to the MOGO
  robot->driveToXY(32, 32);
  robot->TurnAngle(240);
  robot->driveToXY(43, 49);
  wait(20, msec);
  clampFunc();
  robot->turnToXY(21, 46);
  score();
    wait(100,msec);

  robot->driveToXY(25, 46);
    wait(100,msec);
    robot->TurnAngle(0);
wait(100,msec);
  robot->driveToXY(60, 46);  
 }

void auton_red_right() {
robot->default_drive_exit_error = 1;
  robot->default_drive_max = 6;
  robot->default_heading_max = 10;


  // take middle ring
  enableBypass(); 
  lift_intake();
  robot->DriveDistance(9, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(20,msec);
  lift_intake();
  intake_spin_fwd();
  wait(400,msec);
  intake_stop();
  robot->DriveDistance(12, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(40,msec);
  robot->TurnAngle(90);


  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - redStakeApproachDist));
  wait(20,msec);
  // score two rings
  score();
  wait(600,msec);
  score();
  wait(20, msec);
  // drive to the MOGO
  robot->driveToXY(108, 32);
  robot->TurnAngle(305);
  robot->driveToXY(97, 49);
  wait(20, msec);
  clampFunc();
  robot->turnToXY(118, 46);
  score();
    wait(100,msec);

  robot->driveToXY(118, 46);
    wait(100,msec);
    robot->TurnAngle(180);
wait(100,msec);
  robot->driveToXY(80, 46);  
 
}


void auton_red_left()
{
  robot->default_drive_exit_error = 1;
  robot->default_drive_max = 6;
  robot->default_heading_max = 10;


  // take middle ring
  enableBypass(); 
  lift_intake();
  robot->DriveDistance(9, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(20,msec);
  lift_intake();
  intake_spin_fwd();
  wait(400,msec);
  intake_stop();
  robot->DriveDistance(12, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(40,msec);
  robot->TurnAngle(90);


  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - redStakeApproachDist));
  wait(20,msec);
  // score two rings
  score();
  wait(600,msec);
  score();
  wait(20, msec);
  // drive to the MOGO
  robot->driveToXY(32, 32);
  robot->TurnAngle(240);
  robot->driveToXY(43, 49);
  wait(20, msec);
  clampFunc();
  robot->turnToXY(21, 46);
  score();
    wait(100,msec);

  robot->driveToXY(25, 46);
    wait(100,msec);
    robot->TurnAngle(90);
    wait(20,msec);

      robot->default_drive_max = 6;
      float X1 = robot->getX();
      float Y1 = robot->getY();
  robot->driveToXY(X1, Y1+14);
  wait(300,msec);
  robot->DriveDistance(-10, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(40,msec);
 robot->TurnAngle(109);
 wait(40,msec);
   robot->DriveDistance(10, 1.5, 0, 8, 1, 2, 0, 5, 750);
wait(300,msec);
   robot->TurnAngle(10);
    wait(40,msec);
   robot->default_drive_max = 10;
      robot->DriveDistance(21, 1.5, 0, 8, 1, 2, 0, 5, 1450);
score();

}


void auton_blue_right()
{
  robot->default_drive_exit_error = 1;
  robot->default_drive_max = 6;
  robot->default_heading_max = 10;


  // take middle ring
  enableBypass(); 
  lift_intake();
  robot->DriveDistance(9, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(20,msec);
  lift_intake();
  intake_spin_fwd();
  wait(400,msec);
  intake_stop();
  robot->DriveDistance(10, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(40,msec);
  robot->TurnAngle(90);


  float distToField = DistanceSensor.objectDistance(inches);
  wait(20,msec);
  robot->DriveDistance(-(distToField - redStakeApproachDist));
  wait(20,msec);
  // score two rings
  score();
  wait(600,msec);
  score();
  wait(20, msec);
  // drive to the MOGO
  robot->driveToXY(108, 32);
  robot->TurnAngle(305);
  robot->driveToXY(97, 50);
  wait(105, msec);
  clampFunc();
  robot->turnToXY(114, 46);
  score();
    wait(100,msec);

  robot->driveToXY(114, 46);
    wait(100,msec);
   robot->TurnAngle(90);
    wait(50,msec);
      robot->default_drive_max = 6;
      float X1 = robot->getX();
      float Y1 = robot->getY();
  robot->driveToXY(X1, Y1+14);
  wait(300,msec);
    robot->driveToXY(X1, Y1);
  wait(40,msec);

   robot->TurnAngle(64);
   robot->DriveDistance(14, 1.5, 0, 8, 1, 2, 0, 5, 750);
  wait(300,msec);
   robot->TurnAngle(170);
    wait(40,msec);
   robot->default_drive_max = 10;
      robot->DriveDistance(21, 1.5, 0, 8, 1, 2, 0, 5, 1450);
score();


}

void test_auton() {
  //robot->DriveDistance(30, 60);

}

void skills() {
robot->default_drive_exit_error = 2;
robot->default_drive_max = 6;
robot->default_heading_max = 10;

// do not score blue rings
enableBypass();

// score alliance stake
score();
wait(600, msec);
score();

//drive to take first MoGo
robot->driveToXY(75,25);
wait(30, msec);
//robot->TurnAngle(180);
robot->turnToXY(60, 25); // +1 inch for controller exit error correction
wait(30, msec);
robot->DriveDistance(-14, 1.5, 0, 8, 1, 2, 0, 5, 750);
wait(40, msec);
clampFunc();
wait(20, msec);
robot->turnToXY(88,47);
wait(20, msec);
score();
wait(20, msec);
robot->driveToXY(93,52); // was 98
wait(20, msec);
robot->driveToXY(127,70);
wait(20, msec);
robot->driveToXY(118,69); // was 63
wait(500, msec); // wait before turning
robot->turnToXY(118,12);
wait(20, msec);
//robot->default_drive_max = 6;
robot->driveToXY(118,16);//was 118, 38

wait(200, msec);
robot->driveToXY(115,35);
robot->turnToXY(129,24);
robot->driveToXY(129,24);
wait(200, msec);

robot->DriveDistance(-8, 1.5, 0, 8, 1, 2, 0, 5, 750);
wait(20, msec);
robot->TurnAngle(233);
wait(20, msec);
robot->TurnAngle(117);

robot->DriveDistance(-22, 1.5, 0, 8, 1, 2, 0, 5, 1000);
wait(20, msec);
clampFunc();
score();
robot->default_drive_max = 7;
wait(20, msec);
robot->driveToXY(65,26);
wait(100, msec);
robot->turnToXY(140, 26);
wait(20, msec);
robot->default_drive_max = 5;
robot->driveToXY(46,26);

wait(200, msec);
clampFunc();
wait(200, msec);
robot->default_drive_max = 6;
robot->turnToXY(51,46);
score();
wait(20, msec);
robot->driveToXY(46 , 48);
wait(20, msec);
robot->driveToXY(10,75);
wait(20, msec);
robot->driveToXY(22,73);
wait(20, msec);
robot->turnToXY(22,43);
wait(20, msec);
robot->driveToXY(22,16);
wait(200, msec);
robot->driveToXY(25,35); 
wait(20, msec);

robot->turnToXY(10,24);
wait(20, msec);
robot->driveToXY(10,24);
wait(20, msec);
robot->DriveDistance(-8, 1.5, 0, 8, 1, 2, 0, 5, 750);
wait(20, msec);
robot->TurnAngle(120);
wait(20, msec);
robot->TurnAngle(57);
robot->DriveDistance(-19, 1.5, 0, 8, 1, 2, 0, 5, 1000);
wait(20, msec);
clampFunc();
wait(20, msec);
score();
stopWhenColorSeen();
robot->driveToXY(66,66);
score();
robot->driveToXY(82,82);
//score();
intake_spin_fwd();
robot->driveToXY(96,96);
wait(200, msec);
intake_stop();
robot->TurnAngle(315);
wait(20, msec);
robot->driveToXY(82, 106);
wait(20, msec);
robot->default_drive_max = 5;
robot->driveToXY(70, 122);
wait(20, msec);
robot->default_drive_max = 6;
clampFunc();
wait(100, msec);
score();
robot->turnToXY(47, 96);
wait(20, msec);
robot->driveToXY(47, 96);
}


void autonomous(void) {
autonEnabled = true;
Controller1.Screen.clearLine(3);

switch(autonId)
{
  case 1: {
    setAlliance(RED);
    robot->setStartingPoint(55, 12, 45);
    vex::task Position(updatePos);
    auton_red_left();
  break;
  }
  case 2: {
    setAlliance(RED);
    robot->setStartingPoint(85, 10, 135);
    vex::task Position(updatePos);
    auton_red_right();
  break;
  }
  case 3: {
    setAlliance(BLUE);
    robot->setStartingPoint(55, 12, 45);
    vex::task Position(updatePos);
    auton_blue_left();
  break;
  }
  case 4: {
    setAlliance(BLUE);
    robot->setStartingPoint(85, 10, 135);
    vex::task Position(updatePos);
    auton_blue_right();
  break;
  }
  case 5: {
    setAlliance(RED);
    robot->setStartingPoint(70, 12, 90);
    vex::task Position(updatePos);
    test_auton(); 
  break;
  }
  case 6: {
    setAlliance(RED);
    robot->setStartingPoint(70, 11, 90);
    vex::task Position(updatePos);
    skills(); 
  break;
  }
  
}

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
    float turn = Controller1.Axis1.position() * 0.7 ;

    if (fabs(throttle) < 5) {throttle = 0; }; 
    if (fabs(turn) < 5) { turn = 0; }

    robot->SetBrake(coast);

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
    Controller1.ButtonA.pressed(enableBypass);
    // Controller1.ButtonUp.pressed(liftRamp);
  
  
   // start debug output;
  vex::task Debug(ShowMeInfo);
  vex::task Stop(ColorSensing);
  
  // Run the pre-autonomous function.
  pre_auton();

  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  //autonomous();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
