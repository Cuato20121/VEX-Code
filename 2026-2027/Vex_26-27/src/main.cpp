#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>




#include "vex.h"


using namespace vex;







// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))


#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS




/*----------------------------------------------------devices config---------------------------------------------------- */


brain Brain;

controller Controller1 = controller(primary);

// arm Motors
motor MotorArmLeft = motor(PORT2, ratio36_1, true);
motor MotorArmRight = motor(PORT10, ratio36_1, false);

motor_group ArmMotorGroup = motor_group(MotorArmLeft, MotorArmRight);

// Drive Motors
motor DriveFR = motor(PORT18, ratio18_1, true);
motor DriveBR = motor(PORT19, ratio18_1, true);
motor DriveFL = motor(PORT1, ratio18_1, false);
motor DriveBL = motor(PORT8, ratio18_1, false);

motor_group DriveRight = motor_group(DriveFR, DriveBR);
motor_group DriveLeft = motor_group(DriveFL, DriveBL);

// Rotation sensors
rotation RotationRight = rotation(PORT11, false);
rotation RotationLeft = rotation(PORT20, false);

// Pneumatics
pneumatics ClawPistonLeft = pneumatics(Brain.ThreeWirePort.A);
pneumatics ClawPistonRight = pneumatics(Brain.ThreeWirePort.B);


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
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

void motorinit(){
  MotorArmLeft.setStopping(hold);
  MotorArmRight.setStopping(hold);


  DriveRight.setVelocity(0,percent);
  DriveLeft.setVelocity(0,percent);
}


/*----------------------------------------------------controller input---------------------------------------------------- */


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

// define variables used for controlling motors based on controller inputs
bool Controller1XBButtonsControlMotorsStopped = true;
bool Controller1DriveAxisMotorsStopped = true;

// claw togel
bool ClawOn = false;
bool BeenPresed = false;
bool ButtonAState;


// dead zone value
int dz = 10;

int LeftStickVertPos;
int LeftStickHorzPos;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonX/ButtonB status to control ArmMotorGroup
     
      if (Controller1.ButtonX.pressing()) { // logic for arm lift
        ArmMotorGroup.spin(forward);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonB.pressing()) {
        ArmMotorGroup.spin(reverse);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (!Controller1XBButtonsControlMotorsStopped) {
        ArmMotorGroup.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1XBButtonsControlMotorsStopped = true;
      }


      LeftStickVertPos = Controller1.Axis3.position();
      LeftStickHorzPos = Controller1.Axis4.position();


      // dead zone controlls, axis3 is up/down, axis4 is right left on left stick
      if (LeftStickVertPos > dz || LeftStickVertPos < -dz || LeftStickHorzPos > dz || LeftStickHorzPos < -dz ){
        // motor percentage logic
        if (Controller1DriveAxisMotorsStopped == true) // turns motors on
        {
          Controller1DriveAxisMotorsStopped = false;
          DriveRight.spin(forward);
          DriveLeft.spin(forward);
        }


        DriveRight.setVelocity(LeftStickVertPos - LeftStickHorzPos, percent);
        DriveLeft.setVelocity(LeftStickVertPos + LeftStickHorzPos, percent);


       
      } else{
        if (!Controller1DriveAxisMotorsStopped){ // turns motors off to save instructions
            DriveRight.stop();
            DriveLeft.stop();
        }
        Controller1DriveAxisMotorsStopped = true;
      }


    }
    // claw togle logic
    ButtonAState = Controller1.ButtonA.pressing();

    if (ButtonAState == true && BeenPresed == false)
    {
        ClawOn = true;
        BeenPresed = true;
        
    } else if (ButtonAState == false && BeenPresed == true)
    {
        ClawOn = false;
        BeenPresed = false;
    }
    
    if (ClawOn == true) // open and closed are subject to change depending on valve config
    {
        ClawPistonLeft.close();
        ClawPistonRight.close();
    } else 
    {
        ClawPistonLeft.open();
        ClawPistonRight.open();   
    }

    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}


task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);


/*----------------------------------------------------Odometry code---------------------------------------------------- */


// global variables for odometry cordinates
float Wheelx = 0; // in
float Wheely = 0;
float WheelTheta = 0; // rad from -pi(left) to pi(right)


double WheelWidth = 2.75; // in (distance between the two wheels)
double WheelSize = 2.75;


int OdometryUpdateCycle() {
  double PreviousRightEnc = 0.0;
  double PreviousLeftEnc = 0.0;
  double DegreeMultFactor = (M_PI * 2.75) / 360; // factor for degrees to distance


  while (true){
    double DeltaRight = (RotationRight.position(degrees) - PreviousRightEnc) * DegreeMultFactor; // distance traveled by wheels
    double DeltaLeft = (RotationLeft.position(degrees) - PreviousLeftEnc) * DegreeMultFactor;
    PreviousRightEnc = RotationRight.position(degrees);
    PreviousLeftEnc = RotationLeft.position(degrees);


    double DeltaTheta = (DeltaRight - DeltaLeft) / WheelWidth; // step theta


    double AvgDistance = (DeltaRight + DeltaLeft) / 2;


    double MidTheta = WheelTheta + (DeltaTheta / 2); // mid theta more accurate


    double Deltax = AvgDistance * cos(MidTheta); // step distance
    double Deltay = AvgDistance * sin(MidTheta);


    Wheelx = Wheelx + Deltax; // eval global distances
    Wheely = Wheely + Deltay;
    WheelTheta = WheelTheta + DeltaTheta;


    while (WheelTheta > M_PI) { // makes values a circle
      WheelTheta = WheelTheta + (2 * -M_PI);
    }
    while (WheelTheta < -M_PI) {
      WheelTheta = WheelTheta + (2* M_PI);
    }


    vex::task::sleep(20); // sets up threading


  }
  return 0;
}


void PrintDataToController(){
  while (true)
  {
    Controller1.Screen.print("x: {:.2f} y: {:.2f} 0: {:.2f}", Wheelx, Wheely, WheelTheta);
    wait(100, msec);
  }
}



#pragma endregion VEXcode Generated Robot Configuration


// Include the V5 Library
#include "vex.h"
 
// Allows for easier use of the VEX Library
using namespace vex;


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  motorinit();
  thread odometryThread = thread(OdometryUpdateCycle);
  thread screenupdate = thread(PrintDataToController);
 // whenStarted1();
 while (true){
  wait(100, msec);
 }
}





