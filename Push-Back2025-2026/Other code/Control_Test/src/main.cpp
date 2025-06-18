/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       skipp                                                     */
/*    Created:      6/17/2025, 1:21:37 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;



// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here

// define controller config
controller Controller1 = controller(primary);
bool tankStyle = true;
int deadZone = 5;
int pollingRate = 20; // in msec

motor beltMotor = motor(PORT1, ratio18_1, false);
motor intakeMotor = motor(PORT2, ratio18_1, false);
motor LeftDrive = motor(PORT4, ratio18_1, false); // the bools at the end of the statment can be changed
motor RightDrive = motor(PORT3, ratio18_1, true);
//Left motor, Right motor, wheel circumfrance (4 in wheels), track width (Right-Left), wheel base(Up-Down), unit, ext. gear ratio
drivetrain Drivetrain = drivetrain(LeftDrive, RightDrive, 319.18, 300, 300, mm, 1);

// Control sensors and values for thoes sensors
bumper bumper1 = bumper(Brain.ThreeWirePort.G);
int numberOfBumps;
optical optical1 = optical(PORT6);

// define whether remote control should be enabled 
bool RemoteControlCodeEnabled = true;
bool leftStoped = true;
bool rightStoped = true;
// for intake and belt system
int beltValue = 0; // values are 0-stopped 1-forward(torward top) 2-Backward(torward bottom)
int intakeValue = 0; 

int controller1_handaler(){
    // declare variables for intilesence
    int speedOfLeft;
    int speedOfRight;
    while (1)
    {
        if (RemoteControlCodeEnabled == true)
        {
            if (tankStyle == true) // if true both sticks are used if not only left stick is used
            {
                // decide what the value of each motor should be used values from the controller
                speedOfLeft = Controller1.Axis3.position() + Controller1.Axis1.position();
                speedOfRight = Controller1.Axis3.position() - Controller1.Axis1.position();
            } else 
            {
                speedOfLeft = Controller1.Axis3.position() + Controller1.Axis4.position();
                speedOfRight = Controller1.Axis3.position() - Controller1.Axis4.position();
            }
            // decides if the controler is inside the dead zone
            if (speedOfRight < deadZone && speedOfRight > deadZone - (deadZone * 2))
            {
                if (rightStoped == false)
                {
                    RightDrive.stop();
                    // tells the program that the right side is stoped
                    rightStoped = true;
                }  
            } else 
            {
                // tells the program that right should be moving
                rightStoped = false;
            }

            // decides if the controler is inside the dead zone
            if (speedOfLeft < deadZone && speedOfLeft > deadZone - (deadZone * 2))
            {
                if (leftStoped == false)
                {
                    LeftDrive.stop();
                    // tells the program that the left side is stoped
                    leftStoped = true;
                }  
            } else 
            {
                // tells the program that left should be moving
                leftStoped = false;
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
            
            if (rightStoped == false)
            {
                RightDrive.setVelocity(speedOfRight, percent);
                RightDrive.spin(forward);
            }
            
            if (rightStoped == false)
            {
                RightDrive.setVelocity(speedOfRight, percent);
                RightDrive.spin(forward);
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



            // wait before checking again
            wait(pollingRate, msec);
            

        }
        
    }
return 0;
}
// sets threaded task
task controller1_task(controller1_handaler);

int batteryControllerUpdate(){
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Batt-Volt:%f Amp:%f", Brain.Battery.voltage(volt), Brain.Battery.current(amp));
    wait(10,seconds);
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
    Brain.Screen.drawRectangle(1,1,5,4);
}

int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );
   
    while(1) {
        // update sensor values
        bumper1.pressed(bumperUpdate);
        colorUpdate();

        // update screen

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
