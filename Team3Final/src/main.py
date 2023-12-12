# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       final.py                                                      #
# 	Author:       Ishaan                                                       #
# 	Created:      12/3/2023, 6:48:36 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from math import *

#Default Values
WHEEL_DIAMETER = 4.0
DRIVE_SPEED_RATIO = 0.6
WHEEL_TRACK = 12.0 

K_P_DRIVE = 2.5
K_P_TURN = .1
PI = 3.14159      

#states
ROBOT_STOP = 0
ROBOT_TURN_TO_COLLECT = 1
ROBOT_COLLECT = 2
ROBOT_MOVE_TO_GOAL = 3
ROBOT_ALIGN = 4

SHOOTER_STOP = 0
SHOOTER_RUN = 1

shooter_state = SHOOTER_STOP
robot_state = ROBOT_STOP

next_shooter_state = SHOOTER_STOP
next_robot_state = ROBOT_STOP
not_E_stopped = True

#Instantiating Devices
brain = Brain()
controller = Controller()

gyro = Inertial(Ports.PORT6)
gyro.reset_heading()
gyro.calibrate()

sonic = Sonar(brain.three_wire_port.a)

rangeFinder = Sonar(brain.three_wire_port.g)

SIG_RED_BALL = Signature(1, 8945, 11595, 10270, -1391, -471, -930, 2.500, 0)
SIG_BLUE_BALL = Signature(2, -2259, -1025, -1642, 4097, 9345, 6722, 3.000, 0)
indexer_camera = Vision(Ports.PORT13, 50, SIG_RED_BALL)


#Instantiating motors
frontLeft_motor = Motor(Ports.PORT20, 18_1, True)
frontRight_motor = Motor(Ports.PORT11, 18_1, False)
backLeft_motor = Motor(Ports.PORT10, 18_1, True)
backRight_motor = Motor(Ports.PORT2, 18_1, False)
intake_motor = Motor(Ports.PORT12, 18_1, True)
indexer_motor = Motor(Ports.PORT19, 18_1, True)
shooter_motor = Motor(Ports.PORT18, 18_1, True)
hood_motor = Motor(Ports.PORT17, 18_1, True)

def getHeadingError(targetHeading):
    headingError = targetHeading - gyro.heading()
    #say the heading is 359 degrees and we want to go to 0 degrees, the real difference is 1 degree, but a subtraction results in -359 degrees
    # subtracting that/adding to 360 gives us the true error and corrects for the 360 degree to 0 degree jump
    if (headingError > 180):
        headingError = 360 - headingError
    elif (headingError < -180):
        headingError = 360 + headingError
    return headingError

def driveWGyro(wantedHeading, driveSpeed, distInch = 0):
    direction = -K_P_DRIVE*getHeadingError(wantedHeading)
    if (distInch == 0): #drive w/o input distance
        frontLeft_motor.set_velocity(driveSpeed - direction, RPM)   #direction is a K_P modified value
        backLeft_motor.set_velocity(driveSpeed - direction, RPM)   #direction is a K_P modified value
        frontRight_motor.set_velocity(driveSpeed + direction, RPM)
        backRight_motor.set_velocity(driveSpeed + direction, RPM)
        frontLeft_motor.spin(FORWARD)
        backLeft_motor.spin(FORWARD)
        frontRight_motor.spin(FORWARD)
        backRight_motor.spin(FORWARD)
    else: #drive w/ input distance (ultrasconic PID)
        frontLeft_motor.spin_to_position(distInch / (PI*WHEEL_DIAMETER), TURNS)   #direction is a K_P modified value
        backLeft_motor.spin_to_position(distInch / (PI*WHEEL_DIAMETER), TURNS)   #direction is a K_P modified value
        frontRight_motor.spin_to_position(distInch / (PI*WHEEL_DIAMETER), TURNS)
        backRight_motor.spin_to_position(distInch / (PI*WHEEL_DIAMETER), TURNS)

        
# def auton_3ballcollect():
#     #turn on intake
#     for i in range(5):
#         while rangeFinder.distance(DistanceUnits.IN):
            
#         #move fowards to collect the ball
            #while robot is < x inches from the back wall
                #move forward at the current heading
        #back up
            #while robot is > x inches from the back wall
                #move backwards at the current heading
        #wait
            #wait for 2 seconds

def DetectObject():
    objects = indexer_camera.take_snapshot(SIG_RED_BALL)
    if (objects):
        print(" x: ", indexer_camera.largest_object().centerX, "y: ", indexer_camera.largest_object().centerY, "width", indexer_camera.largest_object().width)
        if (indexer_camera.largest_object().centerX < 250):
            return True
    return False
    
while True:
    frontLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    frontRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    backLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    backRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    frontLeft_motor.spin(FORWARD)
    frontRight_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)

# Check for E-Stop
    if(controller.buttonDown.pressing() and not_E_stopped):
        not_E_stopped = False
        next_shooter_state = shooter_state
        next_robot_state = robot_state
        shooter_state = SHOOTER_STOP
        robot_state = ROBOT_STOP
        print('Emergency Stop')
        brain.screen.clear_screen()
        brain.screen.print_at('Emergency Stop', x = 50, y = 50)

    if(controller.buttonUp.pressing()):
        shooter_state = next_shooter_state
        robot_state = next_robot_state
        not_E_stopped = True
        print('Proceed')
        brain.screen.clear_screen()
        brain.screen.print_at('Proceed' , x = 50, y = 50)

#Shooter State Machine
    if(shooter_state == SHOOTER_STOP):
        indexer_motor.stop(BRAKE)
        shooter_motor.stop(COAST)
        print("Shooter Stopped")

    if(shooter_state == SHOOTER_RUN):
        shooter_motor.spin(FORWARD, 200, RPM)
        for i in range(6):
            indexer_motor.spin_for(FORWARD, 165, DEGREES)
            while(shooter_motor.velocity() < 175 ):
                wait(20)
        shooter_state = SHOOTER_STOP

#Robot State Machine
    if(robot_state == ROBOT_STOP):
        frontLeft_motor.stop(BRAKE)
        frontRight_motor.stop(BRAKE)
        backLeft_motor.stop(BRAKE)
        backRight_motor.stop(BRAKE)

    if(robot_state == ROBOT_TURN_TO_COLLECT):
        #code for turning towards the collection zone
        robot_state = ROBOT_COLLECT

    if(robot_state == ROBOT_COLLECT):
        #code for collecting 5 balls
        robot_state = ROBOT_MOVE_TO_GOAL

    if(robot_state == ROBOT_MOVE_TO_GOAL):
        #code for moving towards the blue target
        robot_state = ROBOT_ALIGN

    if(robot_state == ROBOT_ALIGN):
        #code for aligning the robot with the blue target
        robot_state = ROBOT_STOP
        shooter_state = SHOOTER_RUN


    brain.screen.print_at(shooter_motor.velocity(RPM), x=100, y=200)
    #Indexer Pseudocode
    #If the robot detects a ball, move indexer by x amount, stops otherwise.
    # red_objects = indexer_camera.take_snapshot(SIG_RED_BALL)
    # blue_objects = indexer_camera.take_snapshot(SIG_BLUE_BALL)

    # if(DetectObject()):
    #      print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width)
    #      wait(200)
    #      indexer_motor.spin_for(FORWARD, 150, DEGREES)
    # else:
    #     indexer_motor.spin(FORWARD, 0, RPM)

    # if (red_objects or blue_objects):
    #     if(indexer_camera.largest_object().height * indexer_camera.largest_object().width > 100):
    #         print("here")
    #         print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width) #If the robot detects a ball):
    #         brain.screen.print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width)
    #         indexer_motor.spin_for(FORWARD, 150, DEGREES)
    #     else: 
    #         indexer_motor.spin(FORWARD, 0, RPM)
    # else:
    #     indexer_motor.spin(FORWARD, 0, RPM)
    
    if controller.buttonL2.pressing():
        intake_motor.spin(FORWARD, 200, RPM)
        indexer_motor.spin(REVERSE, 50, RPM)
    elif controller.buttonL1.pressing():
        intake_motor.spin(REVERSE, 200, RPM)
        #indexer_motor.spin(FORWARD, 200, RPM)
    if controller.buttonA.pressing():
        shooter_motor.spin(FORWARD, 200, RPM)
    elif controller.buttonB.pressing():
        intake_motor.spin(FORWARD, 0, RPM)
        indexer_motor.spin(FORWARD, 0, RPM)
        shooter_motor.spin(FORWARD, 0, RPM)
    if controller.buttonX.pressing():
        hood_motor.spin(FORWARD, 0, RPM)
    if controller.buttonY.pressing():
        indexer_motor.spin_for(FORWARD, 165, DEGREES)
    if controller.buttonR1.pressing():
        hood_motor.spin(FORWARD, 10, RPM)
    elif controller.buttonR2.pressing():
        hood_motor.spin(REVERSE, 10, RPM)
        




        
