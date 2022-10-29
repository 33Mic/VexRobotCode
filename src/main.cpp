/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> //std::abs

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Settings (HAVE TO BE TUNED MANUALLY)
double kP = 0.228;
double kI = 0.0;
double kD = 0.1473;
double turnkP = 0.077;
double turnkI = 0.0;
double turnkD = 0.0;


//Autonomous Settings
int desiredValue = 0;
int desiredTurnValue = 0;

int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;

int drivePID(){
  
  while(enableDrivePID){

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftTrackingWheel.resetRotation();
      RightTrackingWheel.resetRotation();
      FrontLeftMotor.resetRotation();
      FrontRightMotor.resetRotation();
      BackLeftMotor.resetRotation();
      BackRightMotor.resetRotation();
    }


    //Get the position of both motors
    int leftSidePosition = LeftTrackingWheel.position(degrees);
    int rightSidePosition = RightTrackingWheel.position(degrees);
    
    int leftSideRotationPosition = (FrontLeftMotor.position(deg) + BackLeftMotor.position(deg))/2;
    int rightSideRotationPosition = (FrontRightMotor.position(deg) + BackRightMotor.position(deg))/2;

    /////////////////////////////////////////////////////////////////////
    //                      Lateral movement PID                       //
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int averagePosition = (leftSidePosition + rightSidePosition)/2;
    //Potential
    error = desiredValue - averagePosition;
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(error);
    //Derivative
    derivative = error - prevError;

    //Integral

    totalError += error;
    // double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    double lateralMotorPower = error * kP + derivative * kD;

    /////////////////////////////////////////////////////////////////////
    //                      Turning movement PID                       //
    /////////////////////////////////////////////////////////////////////

    //Get average of the two motors
    int turnDifference = leftSideRotationPosition - rightSideRotationPosition;

    //Potential
    turnError = desiredTurnValue - turnDifference;

    double turnMotorPower = turnError * turnkP;
    /////////////////////////////////////////////////////////////////////

    LeftDriveTrain.spin(forward, lateralMotorPower + turnMotorPower, percent);
    RightDriveTrain.spin(forward, lateralMotorPower - turnMotorPower, percent);


    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}
///////////////////////////////////////////////////////////////////////////////
//                              Methods Used                                 //
///////////////////////////////////////////////////////////////////////////////

void FrontPneumaticOpen()
{
  FrontPneumatic.open();
  this_thread::sleep_for(100);
}
void FrontPneumaticClose()
{
  FrontPneumatic.close();
  this_thread::sleep_for(100);
}
void BackPneumaticOpen()
{
  BackPneumatic.open();
  this_thread::sleep_for(100);
}
void BackPneumaticClose()
{
  BackPneumatic.close();
  this_thread::sleep_for(100);
}
void ConveyorOn()
{
  ConveyorMotor.spin(forward, 80, percent);
  this_thread::sleep_for(100);
}
void ConveyorOff()
{
  ConveyorMotor.stop();
  this_thread::sleep_for(100);
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
  vex::task billWiTheScienceFi(drivePID);

  resetDriveSensors = true;
  desiredValue = 2000;
  desiredTurnValue = 0;

  vex::task::sleep(1600);
  FrontPneumaticClose();

  resetDriveSensors = true;
  desiredValue = -2000;
  desiredTurnValue = 0;
  
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
double turnImportance = 0.5;
void usercontrol(void) {
  // User control code here, inside the loop
  enableDrivePID = false;
  while (1) {

    ///////////////////////////////////////////////////////////////////////////
    //                             Drivetrain                                //
    ///////////////////////////////////////////////////////////////////////////

    double forwardVal = (Controller1.Axis3.position());
    double turnVal = Controller1.Axis1.position();

    double turnVolts = turnVal / 100 * 12; // Max of 12 volts
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    LeftDriveTrain.spin(forward, forwardVolts + turnVolts, volt);
    RightDriveTrain.spin(forward, forwardVolts - turnVolts, volt);

    ///////////////////////////////////////////////////////////////////////////
    //                              Four Bar                                 //
    ///////////////////////////////////////////////////////////////////////////

    double FourBarPosition = Controller1.Axis2.position();
    if(RightLiftMotor.position(degrees) > 50000)
    {
      FourBarLift.spin(forward, -0.5, volt);
    }
    else if (FourBarPosition > 5 || FourBarPosition < -5){
      FourBarLift.spin(forward, FourBarPosition * 0.12, voltageUnits::volt);
    }
    else{
      FourBarLift.stop(hold);
    }
    ///////////////////////////////////////////////////////////////////////////
    //                               Clamp                                   //
    ///////////////////////////////////////////////////////////////////////////
    Controller1.ButtonR1.pressed(FrontPneumaticOpen); // R1 and R2 to clamp on and off
    Controller1.ButtonR2.pressed(FrontPneumaticClose);
    Controller1.ButtonL1.pressed(BackPneumaticOpen); // L1 and L2 to clamp on and off
    Controller1.ButtonL2.pressed(BackPneumaticClose);

    ///////////////////////////////////////////////////////////////////////////
    //                            Conveyor                                   //
    ///////////////////////////////////////////////////////////////////////////
    Controller1.ButtonUp.pressed(ConveyorOn);
    Controller1.ButtonDown.pressed(ConveyorOff);

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print((FrontLeftMotor.position(degrees) + BackLeftMotor.position(degrees))/2);
    Controller1.Screen.newLine();
    Controller1.Screen.print((FrontRightMotor.position(degrees) + BackRightMotor.position(degrees))/2);
    


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
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
