/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       skipp                                                     */
/*    Created:      6/15/2025, 8:35:19 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// declaring structs

struct Position {
    int x;
    int y;
    int degrees;
};





// A global instance of vex::brain used for printing to the V5 brain screen
//vex::brain       Brain;
brain Brain;
// define your global instances of motors and other devices here

// define motors
motor LeftDrive = motor(PORT4, ratio18_1, false); // the bools at the end of the statment can be changed
motor RightDrive = motor(PORT3, ratio18_1, true);
//Left motor, Right motor, wheel circumfrance (4 in wheels), track width (Right-Left), wheel base(Up-Down), unit, ext. gear ratio
drivetrain Drivetrain = drivetrain(LeftDrive, RightDrive, 319.18, 300, 300, mm, 1);

// definition of the 3 encoders: R-Right L-left B-Back

encoder EncoderR = encoder(Brain.ThreeWirePort.A); // 3 pin pairs are A/B C/D E/F G/H
encoder EncoderL = encoder(Brain.ThreeWirePort.C);
encoder EncoderB = encoder(Brain.ThreeWirePort.E);


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                         Odometry for program control                      */
/*                                                                           */
/*  This segment allows for high level movment by accuratly reading of       */
/*  position relitive to the start position of the robot                     */
/*                                                                           */
/*    ALL OF THE MEASUREMENTS ARE IN millimeters FOR MAXIMUM ACCURACY        */
/*---------------------------------------------------------------------------*/

class odometry {
  public:
    // distance from the tracking point to each respective tracking wheel
    std::array<int, 3> trakingPoint; // order Right, Left, Back

    std::array<int, 3> globalPositon;// global variables in order x, y, and rotation

    int circumference; // the circumference of the tracking wheels in mm

    int toleranceXY; // Decrease value for higher tolerance to low may result in feedback loop on vertical and horizontal movment

    int toleranceRotation; // Decrease value for higher tolerance to low may result in feedback loop on rotational movment

    static odometry* instance; // static instnce pointer

    Position findPos(){ // returns the pos in mm from the starting point
      Position pos;
      float theta = 0;
      int distanceR = round(RottoDis(EncoderR.rotation(turns)));
      int distanceL = round(RottoDis(EncoderL.rotation(turns)));
      int distanceB = round(RottoDis(EncoderB.rotation(turns)));
      // calculate degrees
      theta =  (distanceL - distanceR)/(trakingPoint[0] + trakingPoint[1]); // turing distance into radions
      pos.degrees = round(theta  * (180/M_PI)); // converts radions to degrees and rounds
      pos.degrees = pos.degrees % 360; // filtering multiple and left rotations ex: in:-478 out:118

      if (theta == 0){ // prevents the program from attempting to divde by 0
        pos.x = 0;
        pos.y = distanceR;
      } else {
        pos.x = round(2 * sin(theta/2) * ((distanceB/theta) + trakingPoint[2]));
        pos.y = round(2 * sin(theta/2) * ((distanceR/theta) + trakingPoint[0]));
      }
      
      // round(2 * ((distanceR/theta)+Tr) * sin(theta/2)) 

      return pos;
    }
    void movetooLinear(int x, int y){ // rotates then moves directly to the desiered pos NECCESARY
      // if an incrase in the affect of the adjustments are needed the closer they are to the point 
      // closer to the point the more dramatic the difference
      //int distance;
      int degrees;
      
      
      /* This will be done if more accuracy is needed
      std::array<Position,4> points;
      // find 3 points in between origin and end then add the end as a point
      // mid point from origin to end
      points[1].x = (globalPositon[0] + x) / 2;
      points[1].y = (globalPositon[1] + y) / 2;
      // mid point from origin to mid
      points[0].x = (globalPositon[0] + points[1].x) / 2;
      points[0].y = (globalPositon[1] + points[1].y) / 2;
      // mid point from mid to end
      points[2].x = (points[1].x + x) / 2;
      points[2].y = (points[1].y + y) / 2;
      // add end
      points[3].x = x;
      points[3].y = y;
      */
      // do the rotation
      rotate(findDegreesToPoint(x,y));

      //accelerate
      RightDrive.setVelocity(70,percent);
      RightDrive.spin(forward);
      LeftDrive.setVelocity(70,percent);
      LeftDrive.spin(forward);     


      while (true)
      {
        // update variables
        degrees = findDegreesToPoint(x,y);
        EncoderR.changed(odometry::updateGlobalStatic);
        // distance = round(sqrt(pow((x - globalPositon[0]),2)+pow((y - globalPositon[1]),2)));

        // adjustment loop
        if (degrees + toleranceRotation < globalPositon[2] || degrees - toleranceRotation > globalPositon[2]) // if needed implement distance logic here
        {
          if (rightOrLeft(degrees))
          {
            // if needed implement distance logic here
            // increase rate of speed on left side and decrease on right
            RightDrive.setVelocity(67,percent);
            LeftDrive.setVelocity(73,percent);
          } else
          {
            // if needed implement distance logic here
            // incrase rate of speed on right side and decrease on left
            RightDrive.setVelocity(73,percent);
            LeftDrive.setVelocity(67,percent);
          }
          
        }

        // end condition
        if (globalPositon[0] > x - toleranceXY && globalPositon[0] < x + toleranceXY && globalPositon[1] > y - toleranceXY && globalPositon[1] < y + toleranceXY)
        {
          // stop movment
          LeftDrive.stop();
          RightDrive.stop();
          break;
        }
        
      }
      


    }
    
    // case180 is the tie breaker for which direction to rotate when moving 180 degrees ture is right false is left defaults to true
    void rotate(int degrees, bool case180 = true){// rotates to the desiered degree POSSIBILITY
      

      while (true)
      {
        if (rightOrLeft(degrees, case180) == true)
        {
          RightDrive.spin(reverse);
          LeftDrive.spin(forward);
        } else 
        {
          // continuously turn left
        }
        
        
        //  DECITION -----------------------------
        // two ways to do this
        EncoderR.changed(odometry::updateGlobalStatic); // calling the update global function using the changed method
        //updateGlobal();// just calling update global
        // test to see if degrees are close to needDegree


        // final condition to see if rotation is complete
        if (globalPositon[2] < degrees + toleranceRotation && globalPositon[2] > degrees - toleranceRotation)
        {
          // stop movement
          break;
        } 

        
      }
    }
    //void movetooArc(){ moves from one place to another using an arc POSSIBILITY

    //}
    int findDegreesToPoint(int x, int y){
      int needDegree;// degree that is needed to complete the movement from offset
      int horizontal;
      int vertical;
      // find degrees necessary to turn to
      horizontal =  x - globalPositon[0];
      vertical = y - globalPositon[1];
      needDegree = round(atan(horizontal/vertical));
      
      // reasons made are on paper
      // 4 different cases for which movement offset should be applied to adjust the calculated angel to global position
      if (horizontal >= 0 && vertical >= 0) // no offset
      {
        //needDegree = needDegree;
      } else if (horizontal <= 0 && vertical <= 0) // offset of 180
      {
        needDegree += 180;
      } else if (horizontal <= 0 && vertical >= 0) // offset of -360
      {
        needDegree = 360 - needDegree;
      } else if (horizontal >= 0 && vertical <= 0)// offset of -180
      {
        needDegree = 180 - needDegree;
      }
      return needDegree;
    }
    // test to see which direction the robot should turn to go to a desiered angle returns true for right and false for left
    
    bool rightOrLeft(int degrees, bool case180 = true){
      int turnRightDis; // stores the distance in degrees for each direction to the desired 
      int turnLeftDis;
      // findes distance in rotation
      turnRightDis = (degrees - globalPositon[2]);
      if (turnRightDis < 0)
      {
        turnRightDis += 360;
      }
      turnLeftDis = ((globalPositon[2] + 360) - degrees) % 360;
      // deciding which way to turn
      if (turnLeftDis > turnRightDis)
      {
        return true;
      } else if (turnLeftDis < turnRightDis)
      {
        return false;
      } else // this is the case for a 180 turn
      {
        if (case180 == true)
        {
          return true;
        } else 
        {
          return false;
        }
        }



    }

    int RottoDis(double rotations){ // converts the current encoder value to distance traveled in mm
      int distance = circumference * rotations;
      return distance;
    }
    static void updateGlobalStatic(); // Trampoline function
    void updateGlobal(){ // updates global values with the new rotation mesurement of the encoders

      Position pos = findPos(); // calculates difference in position
      
      globalPositon[2] += pos.degrees;// handles negitive cases 
      if (globalPositon[2] < 0){
        globalPositon[2] += 360;
      }


      globalPositon[0] += pos.x;
      globalPositon[1] += pos.y;
      
      /*
      EncoderR.resetRotation();
      EncoderB.resetRotation();
      EncoderL.resetRotation();
      Brain.Screen.clearLine(1);
      Brain.Screen.clearLine(2);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print("x: %d",globalPositon[0]);
      Brain.Screen.newLine();
      Brain.Screen.print("y:%d", globalPositon[1]);
      Brain.Screen.newLine();
      Brain.Screen.print("Degree:%d",globalPositon[2]);
      wait(20,msec);
      */
      Brain.Screen.setCursor(1,1);
      Brain.Screen.print(EncoderB.rotation(degrees));
      
    }

    
    
};

odometry* odometry::instance = nullptr;

void odometry::updateGlobalStatic(){ // trampoline function
  if (instance){
    instance->updateGlobal(); // call the instance method
  }
}




int main() {

    // sets odometry things
    static odometry odMain; // make it exist outside of pre auton
    odometry::instance = &odMain; // assign the instance
    
    odMain.trakingPoint = {10,10,10}; // set this value to the distance of the encoders to the tracking point R,L,B
    odMain.circumference = 10; // set this value to the circumfrance of the encoder wheeles
    odMain.toleranceXY = 1; // n mm of tolerance
    odMain.toleranceRotation = 1; // n degree of tolerance

    odMain.globalPositon = {0,0,0}; // resets values for new game
    // reset encoders
    EncoderR.resetRotation();
    EncoderB.resetRotation();
    EncoderL.resetRotation();

    //Brain.Screen.printAt( 10, 50, "Hello V5" );
   
    RightDrive.setVelocity(70,percent);
    LeftDrive.setVelocity(70, percent);

    while(1) {

        odMain.updateGlobal();
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
