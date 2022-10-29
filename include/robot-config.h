using namespace vex;

extern brain Brain;

extern controller Controller1;

extern motor FrontLeftMotor;
extern motor FrontRightMotor;
extern motor BackLeftMotor;
extern motor BackRightMotor;
extern motor_group LeftDriveTrain;
extern motor_group RightDriveTrain;

extern motor LeftLiftMotor;
extern motor RightLiftMotor;
extern motor_group FourBarLift;

extern motor ConveyorMotor;

extern pneumatics FrontPneumatic;
extern pneumatics BackPneumatic;

extern encoder LeftTrackingWheel;
extern encoder RightTrackingWheel;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
