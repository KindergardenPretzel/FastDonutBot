/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Oleg Khavroniuk                                           */
/*    Created:      5/10/2024, 4:14:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "driveBase.h"
//#include "comp_debug.h"
#include "PID.h"
#include <memory>
#include <iostream>
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;
//competition_debug Cdebug( Competition );

brain Brain;
controller Controller1 = controller(primary);
digital_out clamp = digital_out(Brain.ThreeWirePort.A);
digital_out intake_lift = digital_out(Brain.ThreeWirePort.B);
digital_out Hang = digital_out(Brain.ThreeWirePort.H);
digital_out bypass = digital_out(Brain.ThreeWirePort.C);
digital_out arm = digital_out(Brain.ThreeWirePort.G);
limit auton_switch = limit(Brain.ThreeWirePort.E);

motor intake = motor(PORT5,ratio6_1,false); 
motor scoring = motor(PORT6,ratio6_1,true);
motor hiStakes = motor(PORT10,ratio36_1,true);

optical eyeball = optical(PORT14);
distance DistanceSensor = distance(PORT7);
rotation StakeElevation = rotation(PORT15);

float power_pct = 0.8;
// define your global instances of motors and other devices here

// Robot Class Constructor
// (Inertial, ForwardTrackingWheel, SideTraFckingWheel, LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor, trackingIntchesPerRevolution)
std::shared_ptr<DriveBase> robot(new DriveBase(PORT13, -PORT11, -PORT12, -PORT1, -PORT2, PORT3, PORT4, 6.28));

bool isBeltSpinning = false;
bool isStopperEnabled = false;
bool autonEnabled = false; // reversed. True - selector is disabled, false - enabled
int autonId = 6;
bool isBypassEnabled = false;
bool stake_enable = true;


void score();

// colors from color sensor
enum Alliance {
  RED = 16711680,
  BLUE = 255
};

// high stake scoring mech positions
enum HiStakesEnum {
  Down,
  Armed,
  Scoring
};

Alliance OWN;
Alliance OPPOSITE;

HiStakesEnum StakeScorePosition;

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

//lifts hang using pneumatic cylynder
void hangRobot(){
  if (!Hang.value()) {
        Hang.set(true);
      }
      else {
        Hang.set(false);
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

// corner clearing arm 
void armMove()
{
  if(arm.value())
  {
    arm.set(false);
  }
  else if(!arm.value())
  {
    arm.set(true);
  }
}

// color sensor based functions: Stopper, Bypass, HighStake mech stop belt
int ColorSensing()
{
  eyeball.setLightPower(50, vex::pct);
  eyeball.setLight(ledState::on);
  int prevVelocity;
  while (true) {
    color detectColor = eyeball.color();
    //std::cout << detectColor << std::endl;
    //vex::wait(10, msec);
    if (eyeball.color() == OWN && isBeltSpinning && StakeScorePosition == Armed)
    {
      prevVelocity = scoring.velocity(pct);
      //scoring.setVelocity(60, pct);
      wait(600, msec);
      score();
    }
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
   vex::wait(5, msec);
  }
  return 0;
}

// enable or disable stopper
void stopWhenColorSeen()
{
  if (!isStopperEnabled) {
    isStopperEnabled = true; 
  }
  else{
    isStopperEnabled = false; 
  }

}

// enable or disable bypass
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
    intake_spin_fwd(70);
    scoring.spin(forward, 80, vex::pct);
    isBeltSpinning = true;
  }
  else 
  {
   scoring.stop();
   intake_stop();
   isBeltSpinning = false;
  };
}

// PID for high stake mech positioning
void hiStakeMechGoToPos(float position, vex::brakeType braking_mode)
{
      PID arm_pid = PID(0.5, 0.00002, 0, 5, 1, 2, 8, 1000);
      do {
        float arm_error = position - StakeElevation.position(vex::deg);
        float volt_arm = arm_pid.calculate(arm_error);
        hiStakes.spin(fwd, volt_arm, vex::volt);
      } while (!arm_pid.isFinished());
      hiStakes.stop(braking_mode);
      wait(100, msec);
}

// button callback for high stake mech
void hiStakeScore(){
    if(StakeScorePosition == Armed){
      hiStakes.setVelocity(40,pct);
      hiStakes.spinFor(115, deg, false);
      wait(700,msec);
      hiStakes.stop(hold);
      StakeScorePosition = Scoring;
    }
    if(StakeScorePosition == Down){
      hiStakeMechGoToPos(11, vex::hold);
      StakeScorePosition = Armed;
    }
    
}

//lower high stake mech
void lowerMech(){
  if (StakeScorePosition == Armed || StakeScorePosition == Scoring) {
      hiStakeMechGoToPos(0, vex::coast);
      StakeScorePosition = Down;
      wait(300, msec);
      //StakeElevation.resetPosition();

    
  }
}

//task that updates the robots position using odometry
int updatePos()
{
    while(true)
    {
        robot->updatePosition();
        this_thread::sleep_for(5);
    }
    return(0);
}


//prints info on the brain screen (x,y,angle, stopper, bypass, clamp)
int ShowMeInfo(){
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor(red);
  float heading_angle;
  while(true) {
    //Brain.Screen.setCursor(2,2);
    //Brain.Screen.print("PosHiStake: %f", StakeElevation.position(vex::deg));

    Brain.Screen.setCursor(3,2);
    Brain.Screen.print("X: %f, Y: %f", robot->getX(), robot->getY());

    Brain.Screen.setCursor(4,2);
    heading_angle = robot->getHeading();
    Brain.Screen.print("Heading: %f", heading_angle);


  
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

// Auton Selector ++/--      
void auton_select() {
  autonId ++;
  if(autonId > 9)
    {
      autonId = 1;
    }
  Brain.Screen.clearLine(8);
  Controller1.Screen.clearLine(3);
}

// Stake/No Stake toggle  
void stake_select() {
  stake_enable = !stake_enable;
}


/*---------------------------------------------------------------------------*/
/*             Pre Autonomous Setup and Auton Selector                       */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // auton selector callbacks
  auton_switch.pressed(auton_select);
  Brain.Screen.pressed(stake_select);

  bypass.set(false);
  setAlliance(RED);
  arm.set(false);
  Brain.Screen.setCursor(4,3);
  Brain.Screen.print("Calibrating Inertial Sensor");
  robot->calibrateInertial();
  Brain.Screen.clearScreen();

  while(!autonEnabled)
  {
    wait(20, msec);
    switch(autonId)
    {

      case 1: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Red Left Qual Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: RLQ");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;
      case 2: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Red Right Qual Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: RRQ");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;
      case 3: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Blue Left Qual Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: BLQ");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;
      case 4: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Blue Right Qual Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: BRQ");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;
      case 5: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Test Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: TST");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;   
      case 6: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Skills Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: SKLS");
        
      break;   
      case 7: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Blue Left Elimination Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: BLE");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;   
      case 8: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("Auton: Red Right Elimination Id: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: RRE");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;   
      case 9: 
        Brain.Screen.setCursor(8,2);
        Brain.Screen.print("VENOM: %d", autonId);
        Controller1.Screen.setCursor(3,2);
        Controller1.Screen.print("A: VENOM");
        Controller1.Screen.setCursor(3,13);
        if (stake_enable) {
          Controller1.Screen.print("STAKE");
        }
        else{
          Controller1.Screen.print("NO STAKE");
        }
      break;   
    }


  }

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous                                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*             blue left side for elimination. ID=7                        */
/*---------------------------------------------------------------------------*/
void auton_blue_left_elimination(bool stake = true) {
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  // do not score blue rings
  enableBypass();
  if (stake) {
  //robot->turnToXY(70,2);
  //wait(20, msec);
  //hiStakeMechGoToPos(170, coast);
  //wait(100, msec);
  //hiStakeMechGoToPos(0, coast);
  //wait(20, msec);
  robot->turnToXY(70,24);
  wait(20, msec);
  lift_intake();
  robot->driveToXY(66, 20, 700);
  wait(20, msec);
  lift_intake();
  wait(10, msec);
  intake_spin_fwd();
  wait(200, msec);
  robot->TurnAngle(308);
  wait(20, msec);
}
  robot->default_drive_max = 6;
  robot->driveToXY(48.8,43.4);
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  wait(20, msec);
  score();
  robot->driveStraightToXY(23, 49);
  wait(20, msec);
  robot->driveStraightToXY(17, 14.5);
  wait(20, msec);
  robot->TurnAngle(230);
  wait(150, msec);
  armMove();
  wait(200, msec);
  robot->driveToXY(25, 20.7);
  wait(200, msec);
  armMove();
  //wait(20, msec);
  //float curr_head = robot->getHeading();
  //robot->TurnAngle(curr_head - 20);
  //robot->DriveDistance(10);
  //wait(20, msec);


}


void test_auton(bool stake = true) {
  }


/*---------------------------------------------------------------------------*/
/*             red right side for elimination. ID=8                          */
/*---------------------------------------------------------------------------*/
void auton_red_right_elimination(bool stake = true){
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  

  // do not score blue rings
  enableBypass();
  if (stake) {
   //robot->turnToXY(70,2); 
   //wait(20, msec);
   //hiStakeMechGoToPos(170, coast);
   //wait(100, msec);
   //hiStakeMechGoToPos(0, coast);
   //wait(20, msec);
   robot->turnToXY(70,24);
   wait(20, msec);
   lift_intake();
   robot->driveToXY(74,20, 700);
   wait(20, msec);
   lift_intake();
   wait(10, msec);
   intake_spin_fwd();
   wait(200, msec);
   robot->TurnAngle(218);
   wait(20, msec);
  }
  robot->default_drive_max = 6;
  robot->driveToXY(91,47); /// change for blue right 
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  robot->TurnAngle(10);
  wait(20, msec);
  score();
  robot->driveToXY(120,50);
  wait(20, msec);
  robot->driveStraightToXY(130,18);
  wait(20, msec);
  robot->TurnAngle(316);
  wait(20, msec);
  armMove();
  wait(200, msec);
  //robot->DriveDistance(-10);
  //wait(500, msec);
  //armMove();

}

/*---------------------------------------------------------------------------*/
/*             red right side for qualification. ID=2                        */
/*---------------------------------------------------------------------------*/
void auton_red_right(bool stake = true) {

  if (stake) {

    enableBypass();
    float max_speed = 9;
    robot->default_drive_exit_error = 2;
    robot->default_drive_max = max_speed;
    robot->default_heading_max = 10;
      //intake_spin_back();
      robot->default_drive_max = 6;
      robot->driveToXY(79, robot->getY());
      wait(20, msec);
      //intake_stop();
      lift_intake();
      robot->turnToXY(70, 22);
      wait(20, msec);
      robot->default_drive_max = 4.5;
      robot->driveToXY(70, 22);
      wait(20, msec);
      intake_spin_fwd();
      lift_intake();
      wait(100, msec);
      robot->default_drive_max = max_speed;
      robot->TurnAngle(90);
      wait(20, msec);
      robot->default_drive_max = 5;
      intake_stop();
      robot->driveToXY(70, 5, 600, true);
      wait(20, msec);
      robot->DriveDistance(2.4);
      wait(20, msec);
      score();
      wait(500, msec);
      score();


      robot->driveToXY(79.3, 32.7);
      wait(20, msec);
      robot->turnToXY(70, 24);
      robot->default_drive_max = 6;

    robot->driveToXY(96.6,52);
    
  
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = 11;
  wait(20, msec);
  score();
  robot->driveStraightToXY(117.7, 46);
  wait(200, msec);
  robot->driveStraightToXY(83,56);
  hangRobot();

}


  if (!stake) {
  
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  // do not score blue rings
  enableBypass();
  
  //robot->turnToXY(70,2);
  //wait(20, msec);
  //hiStakeMechGoToPos(115, coast);
  //wait(100, msec);
  //hiStakeMechGoToPos(0, coast);
  //wait(20, msec);
  robot->turnToXY(70,24);
  wait(20, msec);
  lift_intake();
  robot->driveToXY(74, 20, 700);
  wait(20, msec);
  lift_intake();
  wait(10, msec);
  intake_spin_fwd();
  wait(200, msec);
  robot->TurnAngle(218);
  wait(20, msec);
  
  robot->default_drive_max = 6;
  robot->driveToXY(91.2,43.4);
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  robot->TurnAngle(10);
  wait(20, msec);
  score();
  robot->driveToXY(120,50);
  wait(200, msec);
  robot->driveToXY(84,53);
  score();
  hangRobot();
}
}

/*---------------------------------------------------------------------------*/
/*             blue right side for qualification. ID=4                       */
/*---------------------------------------------------------------------------*/
void auton_blue_right(bool stake = true)
{
 
  if (!stake) {
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  // do not score blue rings
  enableBypass();
  
   //robot->turnToXY(70,2);
   //wait(20, msec);
   //hiStakeMechGoToPos(115, coast);
   //wait(100, msec);
   //hiStakeMechGoToPos(0, coast);
   //wait(20, msec);
   robot->turnToXY(70,24);
   wait(20, msec);
   lift_intake();
   robot->driveToXY(74,20, 700);
   wait(20, msec);
   lift_intake();
   wait(10, msec);
   intake_spin_fwd();
   wait(200, msec);
   robot->TurnAngle(218);
   wait(20, msec);
  
  robot->default_drive_max = 6;
  robot->driveToXY(91.2,43.4);
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  robot->TurnAngle(10);
  wait(20, msec);
  score();
  robot->driveToXY(120,50);
  wait(20, msec);
  robot->turnToXY(119.5,68.3);
  wait(20, msec);
  robot->default_drive_max = 6;

  robot->driveToXY(123,60);
  wait(20, msec);
  robot->driveToXY(119.5,56);
  wait(20, msec);

  robot->TurnAngle(109);
  wait(20, msec);
  robot->driveToXY(116.4,63);
  wait(20, msec);
  robot->default_drive_max = max_speed;

  robot->TurnAngle(180);
  wait(20, msec);
  robot->driveToXY(86,61);
  hangRobot();
  }

  if (stake) {

      enableBypass();
      float max_speed = 9;
      robot->default_drive_exit_error = 2;
      robot->default_drive_max = max_speed;
      robot->default_heading_max = 10;
        //intake_spin_back();
        robot->default_drive_max = 6;
        robot->driveToXY(79, robot->getY());
        wait(20, msec);
        //intake_stop();
        lift_intake();
        robot->turnToXY(70, 22);
        wait(20, msec);
        robot->default_drive_max = 4.5;
        robot->driveToXY(70, 22);
        wait(20, msec);
        intake_spin_fwd();
        lift_intake();
        wait(100, msec);
        robot->default_drive_max = max_speed;
        robot->TurnAngle(90);
        wait(20, msec);
        robot->default_drive_max = 5;
        intake_stop();
        robot->driveToXY(70, 5, 600, true);
        wait(20, msec);
        robot->DriveDistance(2.4);
        wait(20, msec);
        score();
        wait(500, msec);
        score();


        robot->driveToXY(79.3, 32.7);
        wait(20, msec);
        robot->turnToXY(70, 24);
        robot->default_drive_max = 6;

      robot->driveToXY(96.6,52);
      
    
    wait(20, msec);
    intake_stop();
    clampFunc();
    wait(200, msec);
    robot->default_drive_max = 11;
    wait(20, msec);
    score();
    robot->driveStraightToXY(117.7, 46);

    
    robot->turnToXY(119.5,68.3);
    wait(20, msec);
    robot->default_drive_max = 8;
  
    robot->driveToXY(123,60);
    wait(20, msec);
    robot->driveToXY(119.5,56);
    wait(20, msec);
  
    robot->TurnAngle(109);
    wait(20, msec);
    robot->driveToXY(116.4,60);
    wait(20, msec);
    robot->default_drive_max = max_speed;
  
    robot->TurnAngle(180);
    wait(20, msec);
    robot->driveToXY(86,61);
    hangRobot();

  }
}

void auton_blue_venom(bool stake = true)
{
 
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  // do not score blue rings
  /*
  enableBypass();
  robot->default_drive_max = 6;
  robot->TurnAngle(270);
  robot->driveToXY(48.8,43.4);
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  robot->TurnAngle(160);
  wait(20, msec);
  score();
  robot->driveToXY(23,50);
  wait(20, msec);
  robot->driveToXY(51.5,62.4);
  hangRobot();
  wait(300, msec);
  score();
  */
 robot->TurnAngle(270);
  robot->DriveDistance(-30);
}


/*---------------------------------------------------------------------------*/
/*             red left side for qualification and elimination. ID=1         */
/*---------------------------------------------------------------------------*/
void auton_red_left(bool stake = true) {
  
if (stake) {

  enableBypass();
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
    //intake_spin_back();
    robot->default_drive_max = 6;
    robot->driveToXY(61, robot->getY());
    wait(20, msec);
    //intake_stop();
    lift_intake();
    robot->turnToXY(70, 22);
    wait(20, msec);
    robot->default_drive_max = 4.5;
    robot->driveToXY(70, 22);
    wait(20, msec);
    intake_spin_fwd();
    lift_intake();
    wait(100, msec);
    robot->default_drive_max = max_speed;
    robot->TurnAngle(90);
    wait(20, msec);
    robot->default_drive_max = 5;
    intake_stop();
    robot->driveToXY(71.5, 5, 600, true);
    wait(20, msec);
    robot->DriveDistance(2.4);
    wait(20, msec);
    score();
    wait(500, msec);
    score();
    robot->driveToXY(61.6, 34.5);
    wait(20, msec);
    robot->turnToXY(70, 24);
    robot->default_drive_max = 6;
  robot->driveToXY(46.7,47.4);
  

wait(20, msec);
intake_stop();
clampFunc();
wait(200, msec);
robot->default_drive_max = max_speed;
robot->TurnAngle(160);
wait(20, msec);
score();
robot->default_drive_max = 11;
robot->driveToXY(23,50);
wait(20, msec);
robot->turnToXY(20.6,68.3);
wait(20, msec);
robot->driveToXY(21.2,60);
wait(20, msec);
robot->TurnAngle(10);
wait(20, msec);
robot->driveToXY(42,62.5);


}

if (!stake) {
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  // do not score blue rings
  enableBypass();
robot->default_drive_max = 6;
robot->driveToXY(48.8,43.4);
wait(20, msec);
intake_stop();
clampFunc();
wait(200, msec);
robot->default_drive_max = max_speed;
robot->TurnAngle(160);
wait(20, msec);
score();
robot->driveToXY(23,50);
wait(20, msec);
robot->turnToXY(20.6,68.3);
wait(20, msec);
robot->driveToXY(21.2,60);
wait(20, msec);
robot->driveToXY(21,57);
wait(20, msec);
robot->TurnAngle(64);
wait(20, msec);
robot->driveToXY(23.6,63);
wait(20, msec);
robot->TurnAngle(0);
wait(20, msec);
robot->driveToXY(42,62.5);
hangRobot();
}
}


/*---------------------------------------------------------------------------*/
/*                              blue left side qualification. ID=3           */
/*---------------------------------------------------------------------------*/
void auton_blue_left(bool stake = false) {

  if (stake) {
    enableBypass();
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
    //intake_spin_back();
    robot->default_drive_max = 6;
    robot->driveToXY(61, robot->getY());
    wait(20, msec);
    //intake_stop();
    lift_intake();
    robot->turnToXY(70, 22);
    wait(20, msec);
    robot->default_drive_max = 4.5;
    robot->driveToXY(70, 22);
    wait(20, msec);
    intake_spin_fwd();
    lift_intake();
    wait(100, msec);
    robot->default_drive_max = max_speed;
    robot->TurnAngle(90);
    wait(20, msec);
    robot->default_drive_max = 5;
    intake_stop();
    robot->driveToXY(71.5, 5, 600, true);
    wait(20, msec);
    robot->DriveDistance(2.4);
    wait(20, msec);
    score();
    wait(500, msec);
    score();
    robot->driveToXY(61.6, 34.5);
    wait(20, msec);
    robot->turnToXY(70, 24);
    robot->default_drive_max = 6;
  robot->driveToXY(46.7,47.4);

  

wait(20, msec);
intake_stop();
clampFunc();
wait(200, msec);
robot->default_drive_max = max_speed;
robot->TurnAngle(160);
wait(20, msec);
score();
robot->driveToXY(23,50);
wait(300, msec);
robot->driveStraightToXY(57, 55);
hangRobot();

  }

  if (!stake) {
    float max_speed = 9;
    robot->default_drive_exit_error = 2;
    robot->default_drive_max = max_speed;
    robot->default_heading_max = 10;
    
    // do not score red rings
    enableBypass();
  robot->default_drive_max = 6;
  robot->driveToXY(48.8,43.4);
  wait(20, msec);
  intake_stop();
  clampFunc();
  wait(200, msec);
  robot->default_drive_max = max_speed;
  robot->TurnAngle(160);
  wait(20, msec);
  score();
  robot->driveToXY(23,50);
  wait(20, msec);
  robot->driveToXY(51.5,62.4);
  hangRobot();
  wait(300, msec);
  score();
  }
}


/*---------------------------------------------------------------------------*/
/*                              Skills in progress, Max 50?                  */
/*---------------------------------------------------------------------------*/
// Skills. ID=6
void skills() {
float max_speed = 9;
robot->default_drive_exit_error = 2;
robot->default_drive_max = max_speed;
robot->default_heading_max = 10;
// do not score blue rings
enableBypass();
// score alliance stake
score();
wait(600, msec);
score();
robot->default_drive_max = 7;
robot->driveToXY(78,20);
wait(20, msec);
robot->TurnAngle(180);
robot->default_drive_max = 6;
wait(20, msec);
robot->driveToXY(94,25);
wait(20, msec);
clampFunc();
wait(20, msec);
robot->default_drive_max = max_speed;
robot->turnToXY(93.4,46.3);
wait(20, msec);
hiStakeScore();
score();
wait(20, msec);
robot->default_drive_max = 6;
robot->driveToXY(93.4,46.3);
wait(20, msec);
robot->driveToXY(115,70.5);
wait(20, msec);
robot->turnToXY(140,70.8); 
wait(20, msec);
intake_spin_fwd();
robot->driveToXY(132, 70.8, 6, 700, false); //high stake. Actual 72.2 (0.6 more than) should be 2 less
wait(20, msec);
hiStakeScore();
wait(20, msec);
robot->default_drive_max = max_speed;
robot->driveToXY(115,robot->getY());
score();
lowerMech();
wait(20, msec);
robot->turnToXY(116,14);
robot->driveToXY(116,14);
//robot->default_drive_max = max_speed;
wait(20, msec);
robot->driveToXY(115.5,26);
wait(20, msec);
robot->turnToXY(127.5, 23);
robot->driveToXY(127.5, 23);
wait(20, msec);
robot->TurnAngle(100);
wait(20, msec);
robot->driveToXY(132,15);
wait(20, msec);
score();
clampFunc();
wait(20, msec);
// Goal unclamped, drive to the left side
robot->driveToXY(70,22);
wait(20, msec);
robot->turnToXY(94,22);
robot->default_drive_max = 6;
wait(20, msec);
robot->driveToXY(45,22);
robot->default_drive_max = max_speed;
clampFunc();
score();
wait(20, msec);
robot->turnToXY(42.5,41);
wait(20, msec);
hiStakeScore();
wait(20, msec);
robot->default_drive_max = 6;

robot->driveToXY(43.5,41);
intake_spin_fwd();
wait(20, msec);
robot->driveToXY(24, 71.3);
wait(20, msec);
robot->turnToXY(0, 72.5);
//robot->default_drive_max = 6;
intake_spin_fwd();
wait(20, msec);
robot->driveToXY(8.5, 72.5, 6, 500, false); // high stake with timeout
wait(20, msec);
hiStakeScore();
wait(20, msec);
robot->default_drive_max = max_speed;
wait(20, msec);
robot->driveToXY(20, robot->getY());
lowerMech();
wait(20, msec);
score();

robot->turnToXY(22, 12); 
robot->driveToXY(22, 12); 
wait(20, msec);
robot->driveToXY(22, 24); 
wait(20, msec);
robot->turnToXY(8, 22); 
robot->driveToXY(8, 22); 
wait(20, msec);
robot->TurnAngle(76);
wait(20, msec);
robot->driveToXY(8, 15); 
score();
clampFunc();
// goal uclamped, go and collect two more rings
wait(20, msec);
robot->default_drive_max = 10;

// enable stopper and collect first ring
stopWhenColorSeen();
score();
robot->driveToXY(22, 92); 
wait(300, msec);
// turn to second, enable intake and go
robot->turnToXY(49, 96);
wait(100, msec);
intake_spin_fwd();
robot->driveToXY(49, 96); 
wait(20, msec);
robot->TurnAngle(233);
wait(20, msec);
robot->default_drive_max = 6;
robot->driveToXY(70, 121); 
wait(20, msec);
clampFunc();
wait(20, msec);
score();
wait(500, msec);
robot->default_drive_max = 10;
robot->TurnAngle(134);
clampFunc();
robot->driveToXY(17, 131); 
robot->driveToXY(88, 130); 
robot->driveToXY(123, 133); 
wait(20, msec);
robot->driveToXY(94, 94); 
hangRobot();
wait(20, msec);
robot->default_drive_max = 12;
robot->driveStraightToXY(72, 72);

}

/*---------------------------------------------------------------------------*/
/*                              Tested Skills, Max 45                        */
/*---------------------------------------------------------------------------*/

// Skills. ID=6. 45 points working 
void skills45() {
  float max_speed = 9;
  robot->default_drive_exit_error = 2;
  robot->default_drive_max = max_speed;
  robot->default_heading_max = 10;
  
  
  // do not score blue rings
  enableBypass();
  
  // score alliance stake
  score();
  wait(600, msec);
  score();
  robot->default_drive_max = 7;
  // first mogo
  robot->driveToXY(78,20);
  wait(20, msec);
  robot->TurnAngle(180);
  robot->default_drive_max = 6;
  wait(20, msec);
  robot->driveToXY(94,25);
  wait(20, msec);
  clampFunc();
  wait(20, msec);
  robot->default_drive_max = max_speed;
  // enable high stake mech and take the ring. 
  robot->turnToXY(93.4,46.3);
  wait(20, msec);
  hiStakeScore();
  score();
  wait(20, msec);
  robot->driveToXY(93.4,46.3);
  wait(20, msec);
  robot->driveToXY(115,71);
  wait(20, msec);
  robot->turnToXY(129.5,71.5);
  wait(20, msec);
  // intake the ring in front of high stake
  intake_spin_fwd();
  robot->default_drive_max = 6;
  robot->driveToXY(129,71.5);
  wait(20, msec);
  hiStakeScore();
  wait(20, msec);
  robot->default_drive_max = max_speed;
  robot->driveToXY(115,71);
  score();
  lowerMech();
  robot->turnToXY(117.8,93.7);
  robot->driveToXY(117.8,93.7);
  wait(20, msec);
  robot->default_drive_max = 7;
  robot->TurnAngle(275);
  robot->driveToXY(116,14);
  robot->default_drive_max = max_speed;
  wait(20, msec);
  robot->driveToXY(115.5,26);
  wait(20, msec);
  robot->turnToXY(127.5, 23);
  robot->driveToXY(127.5, 23);
  wait(20, msec);
  robot->TurnAngle(100);
  wait(20, msec);
  robot->driveToXY(129,16);
  wait(20, msec);
  score();
  clampFunc();
  wait(20, msec);
  robot->driveToXY(70,22);
  wait(20, msec);
  robot->turnToXY(94,22);
  robot->default_drive_max = 6;
  wait(20, msec);
  robot->driveToXY(45,22);
  robot->default_drive_max = max_speed;
  clampFunc();
  score();
  wait(20, msec);
  robot->turnToXY(42.5,41);
  wait(20, msec);
  hiStakeScore();
  wait(20, msec);
  robot->driveToXY(42.5,41);
  intake_spin_fwd();
  wait(20, msec);
  robot->driveToXY(20, 70);
  wait(20, msec);
  robot->turnToXY(4,72.5);
  robot->default_drive_max = 6;
  intake_spin_fwd();
  wait(20, msec);
  robot->driveToXY(7.5, 72.5); // high stake.
  wait(20, msec);
  hiStakeScore();
  wait(20, msec);
  robot->default_drive_max = max_speed;
  wait(20, msec);
  robot->driveToXY(20, 72);
  lowerMech();
  wait(20, msec);
  score();
  robot->turnToXY(18.3, 92); 
  robot->driveToXY(18.3, 92); 
  wait(20, msec);
  robot->turnToXY(22, 12); 
  robot->driveToXY(22, 12); 
  wait(20, msec);
  robot->driveToXY(22, 24); 
  wait(20, msec);
  robot->turnToXY(8, 22); 
  robot->driveToXY(8, 22); 
  wait(20, msec);
  robot->TurnAngle(67);
  wait(20, msec);
  robot->driveToXY(11, 12); 
  score();
  clampFunc();
  wait(20, msec);
  robot->default_drive_max = 10;
  robot->driveToXY(24, 72); 
  robot->driveToXY(52, 122); 
  robot->turnToXY(17, 134); 
  robot->driveToXY(17, 134); 
  robot->driveToXY(88, 133); 
  robot->driveToXY(118, 140);
  }
  

/*---------------------------------------------------------------------------*/
/*                              Autonomous Competition Callback              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
autonEnabled = true; // exit from auton selector
Controller1.Screen.clearLine(3);

switch(autonId)
{
  case 1: {
    // red left qual & elim
    setAlliance(RED);
    if (stake_enable) {robot->setStartingPoint(57.5, 12, 0);}
    if (!stake_enable) {robot->setStartingPoint(56, 12, 0);}
    vex::task Position(updatePos);
    auton_red_left(stake_enable);
  break;
  }
  case 2: {
    // red right qual
    setAlliance(RED);
    if (stake_enable) {robot->setStartingPoint(82.5, 12, 180);}
    if (!stake_enable) {robot->setStartingPoint(84, 12, 180);}
        vex::task Position(updatePos);
    auton_red_right(stake_enable);
  break;
  }
  case 3: {
    //blue left qual
    setAlliance(BLUE);
    if (stake_enable) {robot->setStartingPoint(57.5, 12, 0);}
    if (!stake_enable) {robot->setStartingPoint(56, 12, 0);}
    vex::task Position(updatePos);
    auton_blue_left(stake_enable);
  break;
  }
  case 4: {
    // blue right qual
    setAlliance(BLUE);
    if (stake_enable) {robot->setStartingPoint(82.5, 12, 180);}
    if (!stake_enable) {robot->setStartingPoint(84, 12, 180);}
    
    vex::task Position(updatePos);
    auton_blue_right(stake_enable);
  break;
  }
  case 5: {
    //test
    setAlliance(RED);
    robot->setStartingPoint(57.5, 12, 0);
    vex::task Position(updatePos);
    test_auton(stake_enable); 
  break;
  }
  case 6: {
    // skills
    setAlliance(RED);
    robot->setStartingPoint(70, 10.5, 90);
    vex::task Position(updatePos);
    skills(); 
  break;
  }
  case 7: {
    // blue left elim
    setAlliance(BLUE);
    if (stake_enable) {robot->setStartingPoint(57.5, 12, 0);}
    if (!stake_enable) {robot->setStartingPoint(56, 12, 0);}
        vex::task Position(updatePos);
    auton_blue_left_elimination(stake_enable);
  break;
  }
  case 8: {
    //red right elim
    setAlliance(RED);
    robot->setStartingPoint(84, 12, 180);
    vex::task Position(updatePos);
    auton_red_right_elimination(stake_enable);
  break;
  }  
  case 9: {
    //blue left qual
    setAlliance(BLUE);
    robot->setStartingPoint(60, 12, 0);
    vex::task Position(updatePos);
    auton_blue_venom(stake_enable);
  break;
  }
}

}

// exponental drive. Pilons version
// 
int curveJoystick(int joystick_input, double t_curve){
  return (std::exp(-t_curve/10)+std::exp((std::abs(joystick_input)-100)/10)*(1-std::exp(-t_curve/10))) * joystick_input;
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
  if (Hang.value()) 
  {
    Hang.set(false);
  }
  robot->SetBrake(coast);
  while (1) {
    
    // new joystick curvatire (Pilons team)
    double turn = curveJoystick(Controller1.Axis1.position(percent), 5.1); 
    double throttle = curveJoystick(Controller1.Axis3.position(percent), 5.1); 
 
    robot->LeftMotors.spin(fwd, throttle+turn, vex::pct);
    robot->RightMotors.spin(fwd, throttle-turn, vex::pct);

    wait(10, msec); 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  StakeElevation.resetPosition();
  StakeScorePosition = Down;
  
  //callback
  Controller1.ButtonL1.pressed(clampFunc);
  Controller1.ButtonL2.pressed(lift_intake);
  Controller1.ButtonR2.pressed(reverseIntake);
  Controller1.ButtonR1.pressed(score);
  Controller1.ButtonY.pressed(stopWhenColorSeen);
  Controller1.ButtonA.pressed(enableBypass);
  Controller1.ButtonLeft.pressed(armMove);
  Controller1.ButtonUp.pressed(hiStakeScore);
  Controller1.ButtonDown.pressed(lowerMech);
  Controller1.ButtonX.pressed(hangRobot); 

   // start debug output;
  vex::task Debug(ShowMeInfo);
  vex::task Stop(ColorSensing);
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
