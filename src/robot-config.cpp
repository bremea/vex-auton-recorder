#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT9, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT2, ratio18_1, true);
motor rightMotorB = motor(PORT10, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain =
    drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor Motor8 = motor(PORT8, ratio36_1, false);
motor Motor7 = motor(PORT7, ratio36_1, false);
controller Controller1 = controller(primary);

bool recording = false;
uint8_t rBuff[1000];

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  Brain.SDcard.loadfile("auton", rBuff, Brain.SDcard.size("auton"));
  std::string ffs = "";
  if (recording) {
    Brain.SDcard.savefile("auton", reinterpret_cast<uint8_t *>(&ffs[0]),
                          ffs.length());
  }

  int pos = 0;
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while (true) {
    int ax1 = Controller1.Axis3.position();
    int ax3 = Controller1.Axis3.position();
    bool bL1 = Controller1.ButtonL1.pressing();
    bool bL2 = Controller1.ButtonL2.pressing();
    bool bR1 = Controller1.ButtonR1.pressing();
    bool bR2 = Controller1.ButtonR2.pressing();

    if (!recording) {
      ax1 = rBuff[pos] - 100;
      pos++;
      ax3 = rBuff[pos] - 100;
      pos++;
    } else {
      ffs += (char)(ax1 + 100);
      ffs += (char)(ax3 + 100);
    }
    Brain.SDcard.appendfile("auton", reinterpret_cast<uint8_t *>(&ffs[0]),
                            ffs.length());
    ffs = "";

    if (RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick
      // axies left = Axis3 + Axis1 right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = ax3 + ax1;
      int drivetrainRightSideSpeed = ax3 - ax1;

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left
        // motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right
        // motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }

      // only tell the left drive motor to spin if the values are not in the
      // deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the
      // deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Motor11
      if (Controller1.ButtonL1.pressing()) {
        Motor7.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Motor7.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Motor7.stop();
        // set the toggle so that we don't constantly tell the motor to stop
        // when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Motor8
      if (Controller1.ButtonR1.pressing()) {
        Motor8.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Motor8.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Motor8.stop();
        // set the toggle so that we don't constantly tell the motor to stop
        // when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(recording ? 20 : 30, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}