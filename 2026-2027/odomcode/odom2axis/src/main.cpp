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
rotation RotationRight = rotation(PORT8, false);

rotation RotationLeft = rotation(PORT9, false);




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

#pragma endregion VEXcode Generated Robot Configuration

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

float Wheelx = 0; // in
float Wheely = 0;
float WheelTheta = 0; // rad from -pi(left) to pi(right)
// "when started" hat block
int whenStarted1() {
  return 0;
}

double WheelWidth = 2.75; // in
double WheelSize = 2.75;

int OdometryUpdateCycle() {
  double PreviousRightEnc = 0.0;
  double PreviousLeftEnc = 0.0;
  double DegreeMultFactor = (M_PI * 2.75) / 360;

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


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  vex::task bgTask(OdometryUpdateCycle);
  whenStarted1();
}