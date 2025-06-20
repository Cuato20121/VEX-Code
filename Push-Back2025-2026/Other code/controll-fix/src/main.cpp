#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor LeftDriveSmart = motor(PORT4, ratio18_1, false);
motor RightDriveSmart = motor(PORT3, ratio18_1, true);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 227, 200, mm, 1);

motor beltMotor = motor(PORT7, ratio18_1, false);
motor intakeMotor = motor(PORT1, ratio18_1, true);

// Control sensors and values for thoes sensors
bumper bumper1 = bumper(Brain.ThreeWirePort.G);
int numberOfBumps;
optical optical1 = optical(PORT6);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
  //Controller1.Screen.clearScreen();
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainNeedsToBeStopped_Controller1 = true;

// for intake and belt system
int beltValue = 0; // values are 0-stopped 1-forward(torward top) 2-Backward(torward bottom)
int intakeValue = 0; 


// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis4
      // right = Axis3 - Axis4
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis4.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis4.position();
      
      // check if the values are inside of the deadband range
      if (abs(drivetrainLeftSideSpeed) < 5 && abs(drivetrainRightSideSpeed) < 5) {
        // check if the motors have already been stopped
        if (DrivetrainNeedsToBeStopped_Controller1) {
          // stop the drive motors
          LeftDriveSmart.stop();
          RightDriveSmart.stop();
          // tell the code that the motors have been stopped
          DrivetrainNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the motors next time the input is in the deadband range
        DrivetrainNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

      if (Controller1.ButtonR1.pressing() == true && Controller1.ButtonL1.pressing() == true)
            {
                beltValue = 2;
                intakeValue = 1;
            } else if (Controller1.ButtonR1.pressing() == true)
            {
                beltValue = 1;
                intakeValue = 1;
            } else if (Controller1.ButtonL1.pressing() == true)
            {
                beltValue = 2;
                intakeValue = 2;
            } else
            {
                beltValue = 0;
                intakeValue = 0;
            }

      if (beltValue == 0)
            {
                beltMotor.stop();
            } else if (beltValue == 1)
            {
                beltMotor.spin(forward);
            } else
            {
                beltMotor.spin(reverse);
            }
            
        if (intakeValue == 0)
            {
                intakeMotor.stop();
            } else if (intakeValue == 1)
            {
                intakeMotor.spin(forward);
            } else
            {
                intakeMotor.spin(reverse);
            }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;


int batteryControllerUpdate(){
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Batt-Volt:%d Amp:%d", round(Brain.Battery.voltage(volt)), round(Brain.Battery.current(amp)));
    wait(10,seconds);
    return 0;
}
task batteryControllerTask(batteryControllerUpdate);

void bumperUpdate(){
    numberOfBumps += 1;
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("bumps: %d",numberOfBumps);

}


void colorUpdate(){
    Brain.Screen.setPenColor(optical1.color());
    Brain.Screen.setFillColor(optical1.color());
    Brain.Screen.drawRectangle(1,1,100,100);
}




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Begin project code
  while(1) {
        // update sensor values
        bumper1.pressed(bumperUpdate);
        colorUpdate();

        // update screen

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
