# region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain = Brain()

# Robot configuration code
left_drive_smart = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
right_drive_smart = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
controller_1 = Controller(PRIMARY)
Claw = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
Arm = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
RangeFinderA = Sonar(brain.three_wire_port.a)

# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)


# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs
controller_1_left_shoulder_control_motors_stopped = True
controller_1_right_shoulder_control_motors_stopped = True
drivetrain_needs_to_be_stopped_controller_1 = False


# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_needs_to_be_stopped_controller_1, controller_1_left_shoulder_control_motors_stopped, controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:

            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis4
            # right = axis3 - axis4
            drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis4.position()
            drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis4.position()

            # check if the values are inside of the deadband range
            if abs(drivetrain_left_side_speed) < 5 and abs(drivetrain_right_side_speed) < 5:
                # check if the motors have already been stopped
                if drivetrain_needs_to_be_stopped_controller_1:
                    # stop the drive motors
                    left_drive_smart.stop()
                    right_drive_smart.stop()
                    # tell the code that the motors have been stopped
                    drivetrain_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the motors next
                # time the input is in the deadband range
                drivetrain_needs_to_be_stopped_controller_1 = True

            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
            # check the buttonL1/buttonL2 status
            # to control Claw
            if controller_1.buttonL1.pressing():
                Claw.spin(FORWARD)
                controller_1_left_shoulder_control_motors_stopped = False
            elif controller_1.buttonL2.pressing():
                Claw.spin(REVERSE)
                controller_1_left_shoulder_control_motors_stopped = False
            elif not controller_1_left_shoulder_control_motors_stopped:
                Claw.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_left_shoulder_control_motors_stopped = True
            # check the buttonR1/buttonR2 status
            # to control Arm
            if controller_1.buttonR1.pressing():
                Arm.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonR2.pressing():
                Arm.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            elif not controller_1_right_shoulder_control_motors_stopped:
                Arm.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_shoulder_control_motors_stopped = True
        # wait before repeating the process
        wait(20, MSEC)


# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

# endregion VEXcode Generated Robot Configuration

# ------------------------------------------
#
# 	Project:      VEXcode Project
#	Author:       ME mf
#	Created:      whenever I decide
#	Description:  Autonomus Test Python Project
#
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
vexcode_brain_precision = 0
vexcode_console_precision = 0
vexcode_controller_1_precision = 0
myVariable = 0
Anothervariable = 0
testfordistance1 = 0
testfordistance2 = 0
distanceofirregularities = []
turnofirregularities = []
turned = 0

def ondriver_drivercontrol_0():
    global myVariable, Anothervariable, testfordistance1, testfordistance2, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    Arm.set_stopping(HOLD)
    brain.screen.print("myVariable")
    brain.screen.print(myVariable, precision=6 if vexcode_brain_precision is None else vexcode_brain_precision)
    brain.screen.print("Anothervariable")
    brain.screen.print(Anothervariable, precision=6 if vexcode_brain_precision is None else vexcode_brain_precision)
    if controller_1.buttonX.pressing():
        drivetrain.set_drive_velocity(Anothervariable, PERCENT)
        Anothervariable = Anothervariable + 1
    if controller_1.buttonB.pressing():
        drivetrain.set_drive_velocity(Anothervariable, PERCENT)
        Anothervariable = Anothervariable + 1
    if controller_1.buttonUp.pressing():
        drivetrain.drive(FORWARD)
    if controller_1.buttonDown.pressing():
        drivetrain.drive(REVERSE)
    if controller_1.buttonLeft.pressing():
        drivetrain.turn(LEFT)
    if controller_1.buttonRight.pressing():
        drivetrain.turn(RIGHT)
    if controller_1.buttonL1.pressing():
        Claw.spin(FORWARD)
    if controller_1.buttonL2.pressing():
        Claw.spin(REVERSE)
    if controller_1.buttonR1.pressing():
        myVariable = myVariable + 1
        Arm.spin_to_position(myVariable, DEGREES)
    elif controller_1.buttonR2.pressing():
        myVariable = myVariable + -1
        Arm.spin_to_position(myVariable, DEGREES)
    else:
        Arm.stop()


def when_started1():
    global myVariable, Anothervariable, testfordistance1, testfordistance2, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    pass


def onauton_autonomous_0():
    global myVariable, turned, distanceofirregularities, turnofirregularities, Anothervariable, testfordistance1, testfordistance2, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    V = 0
    # change velocity amounts to correct one when testing
    drivetrain.set_turn_velocity(1, PERCENT)
    turned = turned + 1
    distance1 = range_finder_a.distance(MM)
    while True:
        distance1 = range_finder_a.distance(MM)
        #change wait time to perfection
        wait(0.5, SECONDS)
        distance2 = range_finder_a.distance(MM)
        # change the values of the inequalities when testing to perfection
        # also a gyroscope might make this easier to track positions
        # also also need to make a thing for error correction to correct for the arena
        if distance1 - distance2 < 50:
            # main loop for recording positions of irregularities in path
            #even numbers and 0 are turn and odd numbers are distance
            turnofirregularities[V] = turned

            distanceofirregularities[V] = distance2
            V = V + 1
            pass
        if turned >= 180: #changeable
            break


        wait(5, MSEC)#this may need changing
    #loop to score
    drivetrain.set_turn_velocity(0, PERCENT)
    # enumerate though the distance and find the smallest one
    for i in len(distanceofirregularities):
        if distanceofirregularities[i]



# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    auton_task_0 = Thread(onauton_autonomous_0)
    # wait for the driver control period to end
    while (competition.is_autonomous() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)
    # Stop the autonomous control tasks
    auton_task_0.stop()


def vexcode_driver_function():
    # Start the driver control tasks
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)

    # wait for the driver control period to end
    while (competition.is_driver_control() and competition.is_enabled()):
        # wait 10 milliseconds before checking again
        wait(10, MSEC)
    # Stop the driver control tasks
    driver_control_task_0.stop()


# register the competition functions
competition = Competition(vexcode_driver_function, vexcode_auton_function)