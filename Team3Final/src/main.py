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
DEGREES_PER_INCH = 28.6

K_P_DRIVE = 2.5
KP_STRAIGHT = 0.1 #PID controller
KP_ALIGN = 0.1
K_P_TURN = .04
PI = 3.14159      

#states
ROBOT_STOP = 0
ROBOT_TURN_TO_COLLECT = 1
ROBOT_COLLECT = 2
ROBOT_MOVE_TO_GOAL = 3
ROBOT_ALIGN = 4
ROBOT_PARK = 5
ROBOT_CONTROLLED = 6

SHOOTER_STOP = 0
SHOOTER_RUN = 1

shooter_state = SHOOTER_STOP
robot_state = ROBOT_CONTROLLED

next_shooter_state = SHOOTER_STOP
next_robot_state = ROBOT_TURN_TO_COLLECT
not_E_stopped = True

#Instantiating Devices
brain = Brain()
controller = Controller()

gyro = Inertial(Ports.PORT4)
gyro.reset_heading()
gyro.calibrate()
while gyro.is_calibrating():
    continue

sonic = Sonar(brain.three_wire_port.e)

leftLineSensor = Line(brain.three_wire_port.c)
rightLineSensor = Line(brain.three_wire_port.d)
# farLineSensor = Line(brain.three_wire_port.d)

SIG_GREENYELLOW = Signature(1, -5213, 353, -2430, -4713, -3865, -4289, 2.2, 0)
camera = Vision(Ports.PORT13, 50, SIG_GREENYELLOW)


SET_TURN_SPEED = 50
RAMP_SPEED = 0
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
    headingError = gyro.heading() - targetHeading
    #say the heading is 359 degrees and we want to go to 0 degrees, the real difference is 1 degree, but a subtraction results in -359 degrees
    # subtracting that/adding to 360 gives us the true error and corrects for the 360 degree to 0 degree jump
    if (headingError > 180):
        headingError = -1 * (360 - headingError)
    elif (headingError < -180):
        headingError = -1 * (360 + headingError)
    return headingError

def leftSideDrive(speedInRPM): #combines the two left side motors into one drive function
    frontLeft_motor.set_velocity(speedInRPM, RPM)
    backLeft_motor.set_velocity(speedInRPM, RPM)
    frontLeft_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    
def rightSideDrive(speedInRPM):#combines the two right side motors into one drive function
    frontRight_motor.set_velocity(speedInRPM, RPM)
    backRight_motor.set_velocity(speedInRPM, RPM)
    frontRight_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)

def driveStop():
    leftSideDrive(0)
    rightSideDrive(0)

def breakTopRamp():
    frontLeft_motor.stop(HOLD)
    frontRight_motor.stop(HOLD)
    backLeft_motor.stop(HOLD)
    backRight_motor.stop(HOLD)

def driveWGyro(wantedHeading, driveSpeed):
    direction = K_P_DRIVE*getHeadingError(wantedHeading)
    leftSideDrive(driveSpeed - direction) #direction is a K_P modified value
    rightSideDrive(driveSpeed + direction)

def deadReckonDrive(distInch, speed):
        frontLeft_motor.spin_for(FORWARD,distInch*DEGREES_PER_INCH*DRIVE_SPEED_RATIO,DEGREES,speed,RPM,False)   #direction is a K_P modified value
        backLeft_motor.spin_for(FORWARD,distInch*DEGREES_PER_INCH*DRIVE_SPEED_RATIO,DEGREES,speed,RPM,False)  #direction is a K_P modified value
        frontRight_motor.spin_for(FORWARD,distInch*DEGREES_PER_INCH*DRIVE_SPEED_RATIO,DEGREES,speed,RPM,False)
        backRight_motor.spin_for(FORWARD,distInch*DEGREES_PER_INCH*DRIVE_SPEED_RATIO,DEGREES,speed,RPM,True)

def teleopDrive():
    leftSideDrive(2* (controller.axis3.position()+controller.axis1.position()))
    rightSideDrive(2* (controller.axis3.position()-controller.axis1.position()))

def driveWSonic(distIn, speed):
    wait(250)
    sonicIntial = sonic.distance(DistanceUnits.IN)
    wait(250)
    print("sonic initial: " + str(sonicIntial))
    initialHeading = gyro.heading()
    while(sonic.distance(DistanceUnits.IN) < distIn or sonic.distance(DistanceUnits.IN) < distIn > 500):
        print("here " + str(sonic.distance(DistanceUnits.IN)))
        driveWGyro(initialHeading, speed)
    print("sonic final: " + str(sonic.distance(DistanceUnits.IN)))
    driveStop()

def turnToHeading(desiredHeading):
    #direction is 1 for clockwise rotation
    #direction is -1 for ccw
    #turning to the right is positive degrees for robotTurnInDegrees!
    headingError = getHeadingError(desiredHeading)
    print("heading error: " + str(headingError))
    while (abs(headingError) > 0.75): #while the error is greater than 0.3 degrees then continue to correct using proportional correction
        headingError = getHeadingError(desiredHeading) #if the robot is turned to the right of the desired heading, -K_P returns a negative value
        leftSideDrive(-1 * SET_TURN_SPEED * K_P_TURN * headingError)
        rightSideDrive(SET_TURN_SPEED * K_P_TURN * headingError)
        #print("heading error: " + str(headingError))
    print("done Turn")
    driveStop()
    #stops to reset velocity values 

def turnDegrees(robotTurnInDegrees):
    gyro.calibrate()
    while gyro.is_calibrating():
        continue
    #turns the robot some amount of degrees
    #direction is 1 for clockwise rotation
    #direction is -1 for ccw
    desiredHeading = robotTurnInDegrees + gyro.heading() #wanted new heading
    turnToHeading(desiredHeading)

# drivePID : Integer or Float, Integer or Float -> None
# RESULT : The robot will set the velocity at the given speed, with adjustments by the error parameter
#          A positive error will turn the robot counter-clockwise, and a negative error will turn the robot clockwise
def drivePID(speed, error):
    frontLeft_motor.set_velocity(speed + error, RPM)
    frontRight_motor.set_velocity(speed + error, RPM)
    backLeft_motor.set_velocity(speed + error, RPM)
    backRight_motor.set_velocity(speed + error, RPM)
    frontLeft_motor.spin(FORWARD)
    frontRight_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)

def DetectObject():
    objects = camera.take_snapshot(SIG_GREENYELLOW)
    if (objects):
        print(" x: ", camera.largest_object().centerX, "y: ", camera.largest_object().centerY, "width", camera.largest_object().width)

    return False
    
def AlignWithTarget():
    if (DetectObject()):
        while camera.largest_object().centerX < 150 or camera.largest_object().centerX > 160:
            objects = camera.take_snapshot(SIG_GREENYELLOW)
            drivePID(0, KP_ALIGN * abs(camera.largest_object().centerX - 155) * -1)

while True:
    controller.screen.print(str(robot_state))
    

# # Check for E-Stop
    if(controller.buttonDown.pressing() and not_E_stopped):
        not_E_stopped = False
        next_shooter_state = shooter_state
        next_robot_state = robot_state
        shooter_state = SHOOTER_STOP
        robot_state = ROBOT_STOP
        print('Emergency Stop')
        brain.screen.clear_screen()
        brain.screen.print_at('Emergency Stop', x = 50, y = 50)

    if(controller.buttonY.pressing()):
        shooter_state = SHOOTER_STOP
        if(robot_state != ROBOT_CONTROLLED):
            robot_state = ROBOT_CONTROLLED
        

#Shooter State Machine
    if(shooter_state == SHOOTER_STOP):
        indexer_motor.stop(BRAKE)
        shooter_motor.stop(COAST)
        intake_motor.stop(COAST)
        print("Shooter Stopped")

    if(shooter_state == SHOOTER_RUN):
        intake_motor.spin(FORWARD, 200)
        shooter_motor.spin(FORWARD, 200, RPM)
        for i in range(15):
            while(shooter_motor.velocity() < 170 ):
                wait(20)
            indexer_motor.spin_for(FORWARD, 165, DEGREES)
        robot_state = ROBOT_PARK
        shooter_state = SHOOTER_STOP

#Robot State Machine
    if(robot_state == ROBOT_STOP):
        frontLeft_motor.stop(BRAKE)
        frontRight_motor.stop(BRAKE)
        backLeft_motor.stop(BRAKE)
        backRight_motor.stop(BRAKE)

    if(robot_state == ROBOT_TURN_TO_COLLECT):
        driveWSonic(10, 50)
        turnDegrees(-90)
        robot_state = ROBOT_COLLECT

    if(robot_state == ROBOT_COLLECT):
        intake_motor.spin(REVERSE, 200, RPM)
        for i in range(3):
            deadReckonDrive(25, 50)
            wait(1000)
            deadReckonDrive(-25, 50)
            wait(1000)
            if (i != 2):
                indexer_motor.spin_for(FORWARD, 150, DEGREES)
            else:
                indexer_motor.spin_for(FORWARD, 75, DEGREES)
        for i in range(2):
            deadReckonDrive(25, 50)
            wait(1000)
            deadReckonDrive(-25, 50)
            wait(1000)
        robot_state = ROBOT_MOVE_TO_GOAL

    if(robot_state == ROBOT_MOVE_TO_GOAL):
    #     #code for moving towards the blue target
        intake_motor.spin(FORWARD, 0)
        turnDegrees(90)
        deadReckonDrive(-30, 50)
        deadReckonDrive(30, 50)
        print("start sonic")
        driveWSonic(40, 50)
        deadReckonDrive(35, 50)
        print("end sonic")
        turnDegrees(90)
        robot_state = ROBOT_ALIGN

    if(robot_state == ROBOT_ALIGN):
    #code for aligning the robot with the blue target
        driveWSonic(21.5, 50)
        robot_state = ROBOT_STOP
        shooter_state = SHOOTER_RUN

    if(robot_state == ROBOT_PARK):
        intake_motor.spin(FORWARD, 0)
        turnDegrees(150)
        deadReckonDrive(-75, 50)
        turnDegrees(-60)
        deadReckonDrive(-65, 100)
        robot_state = ROBOT_CONTROLLED
    
    if(robot_state == ROBOT_CONTROLLED):
        break

while True:
    brain.screen.print_at("velocity" + str(shooter_motor.velocity(RPM)), x=100, y=200)
    brain.screen.print_at("current" + str(shooter_motor.current()), x=100, y=150)
    if controller.buttonL2.pressing():
        intake_motor.set_velocity(200, RPM)
        intake_motor.spin(FORWARD)
    elif controller.buttonL1.pressing():
        intake_motor.set_velocity(200, RPM)
        intake_motor.spin(REVERSE)
    if controller.buttonR1.pressing():
        indexer_motor.spin_for(FORWARD, 165, DEGREES)
    elif controller.buttonR2.pressing():
        indexer_motor.spin_for(REVERSE, 165, DEGREES)
    if controller.buttonA.pressing():
        shooter_motor.set_velocity(200, RPM)
        shooter_motor.spin(FORWARD)
    if controller.buttonB.pressing():
        shooter_motor.stop(COAST)
        intake_motor.stop(BRAKE)
    if controller.buttonX.pressing():
        AlignWithTarget()


    teleopDrive()




#     brain.screen.print_at(shooter_motor.velocity(RPM), x=100, y=200)
#     #Indexer Pseudocode
#     #If the robot detects a ball, move indexer by x amount, stops otherwise.
#     # red_objects = indexer_camera.take_snapshot(SIG_RED_BALL)
#     # blue_objects = indexer_camera.take_snapshot(SIG_BLUE_BALL)

#     # if(DetectObject()):
#     #      print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width)
#     #      wait(200)
#     #      indexer_motor.spin_for(FORWARD, 150, DEGREES)
#     # else:
#     #     indexer_motor.spin(FORWARD, 0, RPM)

#     # if (red_objects or blue_objects):
#     #     if(indexer_camera.largest_object().height * indexer_camera.largest_object().width > 100):
#     #         print("here")
#     #         print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width) #If the robot detects a ball):
#     #         brain.screen.print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width)
#     #         indexer_motor.spin_for(FORWARD, 150, DEGREES)
#     #     else: 
#     #         indexer_motor.spin(FORWARD, 0, RPM)
#     # else:
#     #     indexer_motor.spin(FORWARD, 0, RPM)
    
     