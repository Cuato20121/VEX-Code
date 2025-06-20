# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       skipp                                                        #
# 	Created:      6/20/2025, 12:38:35 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *



brain = Brain()

controller = Controller(PRIMARY)

# motors

leftMotor = Motor(Ports.PORT4,GearSetting.RATIO_18_1, False)
rightMotor = Motor(Ports.PORT3,GearSetting.RATIO_18_1, True)
gps = Gps(Ports.PORT8, -152.40, -203.20, MM, 180)
driveTrain = SmartDrive(leftMotor,rightMotor, gps, 319.19, 227, 200, MM, 1)


intakeMotor = Motor(Ports.PORT1,GearSetting.RATIO_18_1,True)
beltMotor = Motor(Ports.PORT7,GearSetting.RATIO_18_1,False)
# sensors
bumper = Bumper(brain.three_wire_port.g)
bumps = 0
colorSens = Optical(Ports.PORT6)

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain GPS
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("GPS")
    gps.calibrate()
    while gps.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)




drivetrain_needs_to_be_stopped_controller = False

beltValue = 0 # values are 0-stopped 1-forward(torward top) 2-Backward(torward bottom)
intakeValue = 0 

# define a task that will handle monitoring inputs from controller
def rc_auto_loop_function_controller():
    global drivetrain_needs_to_be_stopped_controller, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if gps.is_calibrating():
                leftMotor.stop()
                rightMotor.stop()
                while gps.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis4
            # right = axis3 - axis4
            drivetrain_left_side_speed = controller.axis3.position() + controller.axis4.position()
            drivetrain_right_side_speed = controller.axis3.position() - controller.axis4.position()
            
            # check if the values are inside of the deadband range
            if abs(drivetrain_left_side_speed) < 5 and abs(drivetrain_right_side_speed) < 5:
                # check if the motors have already been stopped
                if drivetrain_needs_to_be_stopped_controller:
                    # stop the drive motors
                    leftMotor.stop()
                    rightMotor.stop()
                    # tell the code that the motors have been stopped
                    drivetrain_needs_to_be_stopped_controller = False
            else:
                # reset the toggle so that the deadband code knows to stop the motors next
                # time the input is in the deadband range
                drivetrain_needs_to_be_stopped_controller = True
            
            if controller.buttonR1 == True and controller.buttonL2 == True:
                beltValue = 2
                intakeValue = 1
            elif controller.buttonR1 == True:
                beltValue = 1
                intakeValue = 1
            elif controller.buttonL2 == True:
                beltValue = 2
                intakeValue = 2
            else:
                beltValue = 0
                intakeValue = 0            

            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_needs_to_be_stopped_controller:
                leftMotor.set_velocity(drivetrain_left_side_speed, PERCENT)
                leftMotor.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_needs_to_be_stopped_controller:
                rightMotor.set_velocity(drivetrain_right_side_speed, PERCENT)
                rightMotor.spin(FORWARD)

            if beltValue == 0:
                beltMotor.stop
            elif beltValue == 1:
                beltMotor.spin(FORWARD)
            else:
                beltMotor.spin(REVERSE)

            if intakeValue == 0:
                intakeMotor.stop
            elif intakeValue == 1:
                intakeMotor.spin(FORWARD)
            else:
                intakeMotor.spin(REVERSE)

            
        # wait before repeating the process
        wait(20, MSEC)




rc_auto_loop_thread_controller = Thread(rc_auto_loop_function_controller)


# Calibrate the Drivetrain
calibrate_drivetrain()


def updateSens():
    brain.screen.set_pen_color(colorSens.color)
    brain.screen.set_fill_color(colorSens.color)
    brain.screen.draw_rectangle(100,300,100,100)
    if bumper.pressing:
        bumps =+ 1
    wait(20, MSEC)

updateSens_thread = Thread(updateSens)

def pickup():
    pass


def score(direction):
    pass



def goTOXY(x,y):
    b = x - gps.x_position(MM)
    c = y - gps.y_position(MM)
    if b == 0 or c == 0:
        pass
    else:
        a = (b*b) + (c*c)
        angle = math.atan2(c,b)
        angle = (angle * 180) / math.pi
        driveTrain.turn_to_heading(angle, DEGREES)
        driveTrain.drive_for(FORWARD, a ,MM)



def autonomous():
    global remote_control_code_enabled 
    remote_control_code_enabled = False
    brain.screen.clear_screen()
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.set_fill_color(Color.WHITE)
    brain.screen.print("autonomous code")
    
    goTOXY(0,0)



    
    
    

def user_control():
    global remote_control_code_enabled 
    remote_control_code_enabled = True
    brain.screen.clear_screen()
    brain.screen.set_pen_color(Color.WHITE)
    brain.screen.set_fill_color(Color.WHITE)
    brain.screen.print("driver control")
    # place driver control in this while loop
    while True:
        wait(20, MSEC)



# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()