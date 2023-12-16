# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       final.py                                                      #
# 	Author:       Ishaan, Rohan, Maya                                                       #
# 	Created:      12/3/2023, 6:48:36 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

#Robot Constants
WHEEL_DIAMETER = 4.0
DRIVE_SPEED_RATIO = 0.6
WHEEL_TRACK = 12.0 
DEGREES_PER_INCH = 28.6

#Proportional Control Constants
K_P_DRIVE = 2.5
K_P_TURN = .04
KP_ALIGN = 0.5

#Speed Constants
SET_TURN_SPEED = 50

#Signatures
SIG_GREENYELLOW = Signature(3, -5167, 269, -2449, -4625, -3895, -4260, 1.200, 0)

#Robot states
ROBOT_STOP = 0
ROBOT_TURN_TO_COLLECT = 1
ROBOT_COLLECT = 2
ROBOT_MOVE_TO_GOAL = 3
ROBOT_ALIGN = 4
ROBOT_PARK = 5
ROBOT_CONTROLLED = 6

#Shooter states
SHOOTER_STOP = 0
SHOOTER_RUN = 1

#Holders for current states
shooter_state = SHOOTER_STOP
robot_state = ROBOT_CONTROLLED

#Holders for next states (used with e-stop)
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

camera = Vision(Ports.PORT13, 50, SIG_GREENYELLOW)
leftLineSensor = Line(brain.three_wire_port.c)
rightLineSensor = Line(brain.three_wire_port.d)
sonic = Sonar(brain.three_wire_port.e)

#Instantiating motors
frontLeft_motor = Motor(Ports.PORT20, 18_1, True)
frontRight_motor = Motor(Ports.PORT11, 18_1, False)
backLeft_motor = Motor(Ports.PORT10, 18_1, True)
backRight_motor = Motor(Ports.PORT2, 18_1, False)
intake_motor = Motor(Ports.PORT12, 18_1, True)
indexer_motor = Motor(Ports.PORT19, 18_1, True)
shooter_motor = Motor(Ports.PORT18, 18_1, True)
hood_motor = Motor(Ports.PORT17, 18_1, True)

# ---- Helper functions commonly used in other functions ----

def leftSideDrive(speedInRPM):
    """
    Combines the two left side motors into one left side drive function.

    Parameters:
    - speedInRPM (int or double): The desired speed for the left side drive motors in RPM.
    - A postive number moves the left side forward, a negative number moves the left side backwards 

    Usage:
    - This function sets the velocity of the front left and back left motors to the specified speed in RPM.
    - It then commands both motors to spin.

    Example:
    ```python
    # Set the left side drive motors to spin at 100 RPM
    leftSideDrive(100)
    ```
    """
    frontLeft_motor.set_velocity(speedInRPM, RPM)
    backLeft_motor.set_velocity(speedInRPM, RPM)
    frontLeft_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    
def rightSideDrive(speedInRPM):
    """
    Combines the two right side motors into one right side drive function.

    Parameters:
    - speedInRPM (int or double): The desired speed for the right side drive motors in RPM.
    - A postive number moves the right side forward, a negative number moves the right side backwards

    Usage:
    - This function sets the velocity of the front right and back right motors to the specified speed in RPM.
    - It then commands both motors to spin.

    Example:
    ```python
    # Set the right side drive motors to spin at 100 RPM
    rightSideDrive(100)
    ```
    """
    frontRight_motor.set_velocity(speedInRPM, RPM)
    backRight_motor.set_velocity(speedInRPM, RPM)
    frontRight_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)

def driveStop():
    """
    Stops the robot from moving

    Usage:
    - Calls the leftSideDrive and rightSideDrive function with an RPM of 0

    Example:
    ```python
    # Stops the robot from moving
    driveStop()
    ```
    """
    leftSideDrive(0)
    rightSideDrive(0)

def driveStraight(speed, error):
    """
    Moves the robot forward or backward by the given speed, with adjustments by the error parameter
    Positive error will turn the robot counter-clockwise, and negative error will turn the robot clockwise

    Usage:
    - This function is typically called in a while loop, to keep then robot going straight

    """
    leftSideDrive(speed - error)
    rightSideDrive(speed + error)

def getHeadingError(targetHeading):
    """
    Returns the error in degrees from the given heading to the current heading

    Parameters:
    - targetHeading(int or double) - the heading we want the robot at

    Usage:
    - Subtracts the wanted heading from the curren heading measured by the gyro
    - Corrects for the 360 degree to 0 degree jump
    - Returns the corrected heading error

    Example:
    ```python
    # Returns the offset in degrees from the current heading to 90 degrees
    getHeadingError(90)
    ```
    """
    headingError = gyro.heading() - targetHeading
    #say the heading is 359 degrees and we want to go to 0 degrees, the real difference is 1 degree, but a subtraction results in -359 degrees
    # subtracting that/adding to 360 gives us the true error and corrects for the 360 degree to 0 degree jump
    if (headingError > 180):
        headingError = -1 * (360 - headingError)
    elif (headingError < -180):
        headingError = -1 * (360 + headingError)
    return headingError

def driveDist(distInch, speed): 
    """
    Drives the robot the given distance in inches, at the given speed, using dead reckoning

    Parameters:
    - distInch(int or double) - the distance (in inches) we want the robot to travel
    - speed(int or double) - the speed (in RPM) that the robot should travel at

    Usage:
    - Calculates the degrees that each drivetrain motor should rotate
    - Commands each motor to spin that many degrees at the given speed 

    Example:
    ```python
    # Drive 12 inches at 50 RPM
    driveDist(12, 50)
    ```
    """ 
    degrees = distInch*DEGREES_PER_INCH*DRIVE_SPEED_RATIO

    frontLeft_motor.spin_for(FORWARD,degrees,speed,RPM,False)
    backLeft_motor.spin_for(FORWARD,degrees,speed,RPM,False)
    frontRight_motor.spin_for(FORWARD,degrees,speed,RPM,False)
    backRight_motor.spin_for(FORWARD,degrees,speed,RPM,True)

def driveFromWall(distInch, speed):
    """
    Drives the robot away from the wall (on the rear side of the robot) 
    until it is the given distance in inches, and at the given speed

    Parameters:
    - distInch(int or double) - the distance (in inches) we want the robot to be from the wall
    - speed(int or double) - the speed (in RPM) that the robot should travel at

    Usage:
    - Uses the ultrasonic sensor to find the distance from the rear wall
    - If the distance is less than the given distance, continue calling the drive straight function
    - Stop the robot once it is at or further from the given distance

    Example:
    ```python
    # Drive 14 inches away from the wall at 150 RPM
    driveDist(14, 150)
    ```
    """ 
    wait(250)
    initialHeading = gyro.heading()
    while(sonic.distance(DistanceUnits.IN) < distInch or sonic.distance(DistanceUnits.IN) < distInch > 500):
        direction = K_P_DRIVE*getHeadingError(initialHeading)
        driveStraight(speed, direction)
    driveStop()


def turnDegrees(degrees):
    """
    Turns the robot the given amount of degrees, positive is clockwise, negative is counter-clockwise

    Parameters:
    - distInch(degrees) - the degrees (in degrees) we want the robot to turn

    Usage:
    -  Gets the current error in the heading using the getHeadingError function
    -  While the heading error is > 3/4 of a degree, continue turning the robot and updating the heading
    -  Uses a PID to slow down the robot as it approaches the correct heading
    -  Stops the robot once it reaches the correct heading

    Example:
    ```python
    # Turn 85 degrees clockwise
    driveDist(85)
    ```

    ```python
    # Turn 45 degrees counter-clockwise
    driveDist(-45)
    ```
    """ 
    #turns the robot some amount of degrees
    #direction is 1 for clockwise rotation
    #direction is -1 for ccw
    desiredHeading = degrees + gyro.heading() #wanted new heading
    headingError = getHeadingError(desiredHeading)
    while (abs(headingError) > 0.75): #while the error is greater than 0.3 degrees then continue to correct using proportional correction
        headingError = getHeadingError(desiredHeading) #if the robot is turned to the right of the desired heading, -K_P returns a negative value
        leftSideDrive(-1 * SET_TURN_SPEED * K_P_TURN * headingError)
        rightSideDrive(SET_TURN_SPEED * K_P_TURN * headingError)
    driveStop()
    #stops to reset velocity values 

def DetectObject():
    """
    Returns true if the camera detects an object with the SIG_GREENYELLOW signature, false otherwise

    Usage:
    -  Requests the vision sensor to filter latest objects to match the SIG_GREENYELLOW signature
    -  If there is objects, print the location and return true
    -  If not, return false

    """
    objects = camera.take_snapshot(SIG_GREENYELLOW, 1)
    if (objects):
        print(" x: ", camera.largest_object().centerX, "y: ", camera.largest_object().centerY, "width", camera.largest_object().width)
        return True

    return False
    
    
def AlignWithTarget():
    """
    Aligns the shooter with the yellow-green target

    Usage:
    -  Uses the DetectObject function to determine if there is a valid target in view
    -  while the center of the target is NOT in the center (+ or - 5 pixels), in the x-direction
        drive the robot in the direction of the center of the target
    -  Stops the robot once it reaches the center
    """ 
    if (DetectObject()):
        print("found object")
        while camera.largest_object().centerX < 150 or camera.largest_object().centerX > 160:
            objects = camera.take_snapshot(SIG_GREENYELLOW)
            print(" x: ", camera.largest_object().centerX, "y: ", camera.largest_object().centerY, "width", camera.largest_object().width)
            driveStraight(0, KP_ALIGN * (camera.largest_object().centerX - 155) * -1)
    else:
        print("no object")

def moveUntilWhite():
    """
    Moves the robot forward until both line trackers have reached a white line

    Usage: 
    - While both line sensors have not detected a white line, the robot will move forward
    using the leftSideDrive and rightSideDrive functions
    - Stops the robot when both line trackers have detected a white line

    """ 
    print(leftLineSensor.value()) # getting an initial reading
    print(rightLineSensor.value())
    wait(250) #delay for the robot to calculate the above lines
    while leftLineSensor.value() > 2000 and rightLineSensor.value() > 2000: #the value of a white line is approximately <= 2000
        #continue moving worwards until we reach a white line
        leftSideDrive(25)
        rightSideDrive(25)
    driveStop()

def teleopDrive():
    """
    Moves the robot using the joycon inputs on the controller
    Usage: 
    - Is called in a while loop during tele-op, to give drivers control of the robot
    - Left joystick controls the speed of the robot, 
    and the right joystick controls the turning adjustments
    
    """ 
    leftSideDrive(2* (controller.axis3.position()+controller.axis1.position()))
    rightSideDrive(2* (controller.axis3.position()-controller.axis1.position()))

#Autonomous while loop
while True:
    #This match-case makes a custom display on the controller
    #It will print the current robot state on the controller, making debugging easier
    #We use a match-case instead of if-else statements since we work with enumerated types
    match robot_state:
        case 0:
            controller.screen.print("Robot is in the ROBOT_STOP state")
        case 1:
            controller.screen.print("Robot is in the ROBOT_TURN_TO_COLLECT state")
        case 2:
            controller.screen.print("Robot is in the ROBOT_COLLECT state")
        case 3:
            controller.screen.print("Robot is in the ROBOT_MOVE_TO_GOAL state")
        case 4:
            controller.screen.print("Robot is in the ROBOT_ALIGN state")
        case 5:
            controller.screen.print("Robot is in the ROBOT_PARK state")
        case 6:
            controller.screen.print("Robot is in the ROBOT_CONTROLLED state")
    

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

        

#Shooter State Machine
    if(shooter_state == SHOOTER_STOP):
        indexer_motor.stop(BRAKE)
        shooter_motor.stop(COAST)
        intake_motor.stop(COAST)
        print("Shooter Stopped")

    if(shooter_state == SHOOTER_RUN):
        intake_motor.spin(FORWARD, 200)
        shooter_motor.spin(FORWARD, 200, RPM)
        for i in range(15): #Run 15 times to ensure all balls in the robot are launched
            while(shooter_motor.velocity() < 170 ):
                wait(20) #If the velocity is not great enough, give it time to speed up
            indexer_motor.spin_for(FORWARD, 165, DEGREES) #Index a ball once the shooter velocity is great enough
        shooter_state = SHOOTER_STOP
        robot_state = ROBOT_CONTROLLED

#Robot State Machine
    if(robot_state == ROBOT_STOP):
        #Brake all drive motors
        frontLeft_motor.stop(BRAKE)
        frontRight_motor.stop(BRAKE)
        backLeft_motor.stop(BRAKE)
        backRight_motor.stop(BRAKE)

    if(robot_state == ROBOT_TURN_TO_COLLECT):
        #Turn to face the collection zone on the left
        driveFromWall(10, 50)
        turnDegrees(-90)
        robot_state = ROBOT_COLLECT

    if(robot_state == ROBOT_COLLECT):
        #Move forwards and backwards to collect balls from the collection zone
        intake_motor.spin(REVERSE, 200, RPM)
        #Collect 3 balls but only fully index the first 2, otherwise do a half index
        for i in range(3):
            driveDist(25, 50)
            wait(1000)
            driveDist(-25, 50)
            wait(1000)
            if (i != 2):
                indexer_motor.spin_for(FORWARD, 150, DEGREES)
            else:
                indexer_motor.spin_for(FORWARD, 75, DEGREES)
        #Collect two balls and don't index at all
        for i in range(2):
            driveDist(25, 50)
            wait(1000)
            driveDist(-25, 50)
            wait(1000)
        robot_state = ROBOT_MOVE_TO_GOAL

    if(robot_state == ROBOT_MOVE_TO_GOAL):
        #code for moving towards the blue target
        intake_motor.spin(FORWARD, 0)
        turnDegrees(90)
        driveDist(-30, 50)
        driveDist(30, 50)
        driveFromWall(40, 50)
        driveDist(35, 50)
        turnDegrees(90)
        robot_state = ROBOT_ALIGN

    if(robot_state == ROBOT_ALIGN):
    #code for aligning the robot with the blue target
        driveFromWall(21.5, 50)
        robot_state = ROBOT_STOP
        shooter_state = SHOOTER_RUN

    if(robot_state == ROBOT_PARK):
        #Cross over the bump under the ramp
        intake_motor.spin(FORWARD, 0)
        turnDegrees(150)
        driveDist(-75, 50)
        turnDegrees(-60)
        driveDist(-65, 100)
        robot_state = ROBOT_CONTROLLED
    
    if(robot_state == ROBOT_CONTROLLED):
        #Once we get here we break out of the autonmous for-loop,
        #putting us in the tele-op for-loop
        break

#Tele-op while loop
while True:
    #Print the velocity and current going into the shooter on the brain
    brain.screen.print_at("velocity" + str(shooter_motor.velocity(RPM)), x=100, y=200)
    brain.screen.print_at("current" + str(shooter_motor.current()), x=100, y=150)
    #Dispense balls with the intake
    if controller.buttonL2.pressing():
        intake_motor.set_velocity(200, RPM)
        intake_motor.spin(FORWARD)
    #Collect balls with the intake
    elif controller.buttonL1.pressing():
        intake_motor.set_velocity(200, RPM)
        intake_motor.spin(REVERSE)
    #Move the indexer forward by 1 index
    if controller.buttonR1.pressing():
        indexer_motor.spin_for(FORWARD, 165, DEGREES)
    #Move the indexer backward by 1 index
    elif controller.buttonR2.pressing():
        indexer_motor.spin_for(REVERSE, 165, DEGREES)
    #Run the flywheel
    if controller.buttonA.pressing():
        shooter_motor.set_velocity(200, RPM)
        shooter_motor.spin(FORWARD)
    #Stop both the intake and the shooter
    if controller.buttonB.pressing():
        shooter_motor.stop(COAST)
        intake_motor.stop(BRAKE)
    #Align the shooter with the target (Sensor-Assisted Delivery)
    if controller.buttonX.pressing():
        AlignWithTarget()
    #Move forwards until the line sensors pick up a white line
    if controller.buttonY.pressing():
        moveUntilWhite()
    #Continuously call this function to give a driver control over the robot
    teleopDrive()