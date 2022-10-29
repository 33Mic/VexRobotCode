#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Controller
controller Controller1 = controller(primary);

// DriveTrain
motor FrontLeftMotor = motor(PORT1, ratio18_1, true);
motor FrontRightMotor = motor(PORT10, ratio18_1, false);
motor BackLeftMotor = motor(PORT11, ratio18_1, true);
motor BackRightMotor = motor(PORT20, ratio18_1, false);
motor_group LeftDriveTrain = motor_group(FrontLeftMotor, BackLeftMotor);
motor_group RightDriveTrain = motor_group(FrontRightMotor, BackRightMotor);

// Lift
motor LeftLiftMotor = motor(PORT2, ratio36_1, false);
motor RightLiftMotor = motor(PORT9, ratio36_1, true);
motor_group FourBarLift = motor_group(LeftLiftMotor, RightLiftMotor);

// Conveyor
motor ConveyorMotor = motor(PORT5, ratio18_1, true);

// Clamp
pneumatics FrontPneumatic = pneumatics(Brain.ThreeWirePort.H);
pneumatics BackPneumatic = pneumatics(Brain.ThreeWirePort.F);

// Shaft Encoders
encoder LeftTrackingWheel = encoder(Brain.ThreeWirePort.A);
encoder RightTrackingWheel = encoder(Brain.ThreeWirePort.C);
bool RemoteControlCodeEnabled = true;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  FrontLeftMotor.setBrake(coast);
  FrontRightMotor.setBrake(coast);
  BackLeftMotor.setBrake(coast);
  BackRightMotor.setBrake(coast);

  FrontPneumatic.open();
  // Nothing to initialize
}