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
// AI Vision Color Descriptions
// AI Vision Code Descriptions
vex::aivision AIVision1(PORT2, aivision::ALL_TAGS);

gps GPS2 = gps(PORT6, 0.00, 185.00, mm, 97);
distance Distance3 = distance(PORT3);
controller Controller1 = controller(primary);
motor MotorShoulder = motor(PORT12, ratio18_1, false);

motor MotorRight1 = motor(PORT1, ratio18_1, false);

motor MotorLeft2 = motor(PORT20, ratio18_1, false);

motor MotorLeft1 = motor(PORT13, ratio18_1, false);

motor MotorRight2 = motor(PORT10, ratio18_1, false);

motor MotorElbow = motor(PORT14, ratio18_1, false);



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



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {Jakson Meek}                                             */
/*    Created:      {05/19/26}                                                */
/*    Description:  Navigation System v0.1                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <vector>
#include <string>
using namespace vex;

struct AprilTagInfo {
    int id;
    double x;
    double y;
    
};

std::vector<AprilTagInfo> tagMap = {
    {0, -24.0, -48.0}, // Bottom-Left element
    {1,  24.0, -48.0}, // Bottom-Right element
    {2,   0.0,   0.0}, // Center field element (0,0)
    {3, -24.0,  48.0}, // Top-Left element
    {4,  24.0,  48.0}  // Top-Right element
};


std::string currentState = "";
bool driverControl;
double desiredHeight;
double angleT;
double distanceT;
double mapX;
double mapY;
double mapHeading;
int selection;
int selectedPins;
bool toggle = false;
double axis3Emu;
std::string selectedGoal;

//==================================//
//=============Utilities============//
//==================================//

//**ORIGIN IS BACK LEFT CORNER FROM BLUE ALLIANCE**//

double angleToTag(int xCenter) {
    const double FOV = 74.0;     
    const double HALF_FOV = FOV / 2.0;
    const double IMG_CENTER = 160.0;

    return ((xCenter - IMG_CENTER) / IMG_CENTER) * HALF_FOV;
}

double distanceToTag(double pixelWidth, double angleDeg) {
    const double realTagWidth = 1.811;   
    const double focalLength = pixelWidth/(2*tan(1.29154/2));  
    double angleRad = angleDeg * M_PI / 180.0;

    return (realTagWidth * focalLength * cos(angleRad)) / pixelWidth;
}

void translateOdom(double angleTag, double distanceTag) {
    AIVision1.takeSnapshot(aivision::ALL_TAGS);
    
    if (AIVision1.objectCount > 0) {
        auto obj = AIVision1.largestObject;
        
        // Find tag in map
        double tagX = 0, tagY = 0;
        bool found = false;
        for(auto& tag : tagMap) {
            if(tag.id == obj.id) {
                tagX = tag.x;
                tagY = tag.y;
                found = true;
                break;
            }
        }

        if (!found) return;

        // 2. FIX: Get the absolute field heading directly from the GPS.
        // The VEX GPS automatically manages its global angle, whether you are Blue or Red.
        double currentHeadingRad = GPS2.heading(rotationUnits::deg) * M_PI / 180.0;
        double angleTagRad = angleTag * M_PI / 180.0;
        
        // 3. FIX: Compute the true global angle of the line pointing from the robot to the tag.
        double absoluteVectorRad = currentHeadingRad + angleTagRad; 

        // 4. FIX: Subtract the vector from the tag's global position to find the robot's global position.
        double robotX = tagX - ((distanceTag+7.28) * cos(absoluteVectorRad));
        double robotY = tagY - ((distanceTag+7.28) * sin(absoluteVectorRad));

        // Update global state (simple weighting)
        if (mapX == 0) {
          mapX = robotX;
        } else {
          mapX = (mapX * 0.8) + (robotX * 0.2);
        }

        if (mapY == 0) {
          mapY = robotY;
        } else {
          mapY = (mapY * 0.8) + (robotY * 0.2);
        }

        mapHeading = GPS2.heading();
    }
}

bool visualCheck(){
  AIVision1.takeSnapshot(aivision::ALL_TAGS);

  if (AIVision1.objectCount > 0) {
    auto obj = AIVision1.largestObject;

    angleT = angleToTag(obj.centerX);
    distanceT = distanceToTag(obj.width, angleT);
    translateOdom(angleT,distanceT);
    return true;
  }
  return false;
}

double getX(){
//instead of (mapX + GPS)/2, use a trust factor
double trust = 0.7; //trust GPS more than calculated tag position
return (mapX * (1.0 - trust)) + (GPS2.xPosition(inches) * trust);
}

double getY(){
double trust = 0.7;
return (mapY * (1.0 - trust)) + (GPS2.yPosition(inches) * trust);
}

void IK(double armHeight){
  //fill in IK algorithm
}

void score(int pinsStacked, std::string stackType){
  //find grab height and replace 6.5
  if(stackType == "Tall"){
    desiredHeight = 8.7+(6.5*pinsStacked);
  }
  else if(stackType == "Medium"){
    desiredHeight = 5.8+(6.5*pinsStacked);
  }
  else if(stackType == "Small"){
    desiredHeight = 3.25+(6.5*pinsStacked);
  }
  else {
    return;
  }
  //orient based on height
  IK(desiredHeight);
}

void switchColor(){
  //Shoulder.spinToPosition(maximumAngle);
  //Shoulder.spinToPosition(minimumAngle);
}

bool isSearching(){
  if (driverControl == false){
    return true;
  }
  return false;
}

void driveTo(double targetX, double targetY) {
    double kP = 5.0; // Tune this
    double minSpeed = 10.0; // Prevent stalling out
    int timeoutMs = 5000; // 5 seconds timeout to break infinite loops
    int startTime = vex::timer::system();

    while(((vex::timer::system() - startTime) > timeoutMs)) {
        double errorX = targetX - getX();
        double errorY = targetY - getY();
        
        // correctly calculate angle using atan2
        double angle = atan2(errorY, errorX) * (180.0 / M_PI);
        double distance = sqrt(errorX*errorX + errorY*errorY); 
        
        // 1. break condition: arrived within 1 foot OR timeout reached
        if(distance < 12.0 || ((vex::timer::system() - startTime) > timeoutMs)) {
            // Drivetrain.stop();
            return; 
        }

        double speed = distance * kP;
        if(speed > 100) speed = 100; // cap at full power
        if(speed < minSpeed) speed = minSpeed;

        // Drivetrain.turnToHeading(angle, degrees); 
        // Drivetrain.drive(forward, speed, rpm);

        wait(20, msec); 
    }
}

void flip(){
  static double lastTime = 0;
  double currentTime = vex::timer::system();
  if(currentTime - lastTime < 250) return; // 250ms debounce
    
  driverControl = !driverControl;
  lastTime = currentTime;
  
}

void pollControls(){


  if(-Controller1.Axis3.position() > 10 || -Controller1.Axis3.position() < -10){
    axis3Emu = -Controller1.Axis3.position();
  }
  else{
    axis3Emu = -Controller1.Axis3.position();
  } 
  
  double throttle =axis3Emu;
  double turn = -Controller1.Axis1.position();

  if(fabs(throttle) < 15.0) throttle = 0.0;
  if(fabs(turn) < 15.0) turn = 0.0; 

  double targetLeftSpeed = throttle + turn;
  double targetRightSpeed = throttle - turn;
    
  if(targetLeftSpeed > 100) targetLeftSpeed = 100;
  if(targetLeftSpeed < -100) targetLeftSpeed = -100;
  if(targetRightSpeed > 100) targetRightSpeed = 100;
  if(targetRightSpeed < -100) targetRightSpeed = -100;

  MotorLeft1.spin(forward, -targetLeftSpeed, percent);
  MotorLeft2.spin(forward, -targetLeftSpeed, percent);
  MotorRight1.spin(forward, targetRightSpeed, percent);
  MotorRight2.spin(forward, targetRightSpeed, percent);

  if(Controller1.ButtonL1.pressing()){
    MotorShoulder.spin(forward, 100, percent);
    MotorElbow.spin(reverse, 100, percent);
  }
  
  if(Controller1.ButtonR1.pressing()){
    MotorShoulder.spin(reverse, 100, percent);
     MotorElbow.spin(forward, 100, percent);
  }
}

void frameController(double x,double y){

  Controller1.Screen.clearScreen();

  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("X: ");
  Controller1.Screen.print(x);
  Controller1.Screen.newLine();
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print("Y: ");
  Controller1.Screen.print(y);

  Brain.Screen.clearScreen();
  
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("X: ");
  Brain.Screen.print(x);
  Brain.Screen.newLine();
  Brain.Screen.setCursor(2,1);
  Brain.Screen.print("Y: ");
  Brain.Screen.print(y);
  wait(0.01,seconds);
}

void mainStateControl(){

  if(isSearching()){

    double tagX = 0, tagY = 0;
    bool found = false;
    for(auto& tag : tagMap) {
      if(tag.id == selection) {
        tagX = tag.x;
        tagY = tag.y;
        found = true;
        break;
      }
    }
    if (!found) return;

    driveTo(tagX,tagY);

    if(visualCheck()){
      currentState = "TAG FOUND";
      //score(selectedPins, selectedGoal);
      //initiate scoring algorithm

    } else{
      currentState = "SEARCHING FOR TAG";
      //continue waiting
    }
  } else{
    currentState = "IDLE";
    pollControls();
  }
}

//==================================//
//==========Utilities End===========//
//==================================//

int main() {
  vexcodeInit();
  driverControl = true;
  GPS2.calibrate();

  Controller1.ButtonR2.pressed(flip);

  while(true){

    mainStateControl();
    frameController(getX(),getY());
    wait(20, msec);
  }
  
}

