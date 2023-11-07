# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       rohan                                                        #
# 	Created:      11/1/2023, 8:57:46 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

left_motor = Motor(Ports.PORT10, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)
rangeFinderFront = Sonar(brain.three_wire_port.g)
rangeFinderRightSide = Sonar(brain.three_wire_port.a)
gy = Inertial(Ports.PORT6)
#clockwise is positive rotation for IMU
gy.reset_heading()
gy.calibrate()


WHEEL_DIAMETER = 4.0
GEAR_RATIO = 5.0
WHEEL_TRACK = 11.0 

K_P_DRIVE = 2.5
K_P_TURN = .1
SET_WALL_FOLLOW_SPEED = 120         # RPM
SET_WALL_DISTANCE = 10            # inches for wall following
TURN_ANGLE_AT_WALL = -90           # turn left angle in degrees (right = positive)
SET_DISTANCE_TO_START_FIRST_TURN =  10    # inches from wall in front to begin turn
SET_DISTANCE_TO_START_SECOND_TURN = 4
SET_TURN_SPEED = 70
SPIN_AROUND_ANGLE = 180
currHeading = 0

# drive function - For negative values of direction, the robot turns right, and for positive
# values of direction, the robot turns right.  For values of direction with small magnitudes, the
# robot gradually turns.  For values of direction with large magnitudes, the robot turns more
# quickly.

def drive(speed, direction):
   left_motor.set_velocity(speed - direction, RPM)
   right_motor.set_velocity(speed + direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)
def turn(speed, direction):
    left_motor.set_velocity(speed*direction, RPM)
    right_motor.set_velocity(-1 * speed*direction, RPM)
    # print("left:" + str(speed*direction))
    # print("right:"+ str(-1 * speed*direction))
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)
def stop():
    drive(0, 0)
   
def convertHead(heading):
    if (heading > 180):
        heading -= 360
    return heading

# Function to wall follow at set distance from wall

def getError(targetHeading):
    headingError = targetHeading - gy.heading()
    if (headingError > 180):
        headingError = 360 - headingError
    elif (headingError < -180):
        headingError = 360 + headingError
    return headingError

def driveAtHeading(currHeading):
    headingError = getError(currHeading) #if the robot is turned to the right and the left wheel is too fast (pos)
    #-K_P * error returns a negative error, need to make the right wheel go faster
    # print("error: " + str(headingError))
    drive(SET_WALL_FOLLOW_SPEED, -K_P_DRIVE*headingError)

# Function to turn BaseBot for some number of degrees

def turnInPlace(robotTurnInDegrees, currHeading):
    #turning to the right is positive degrees for robotTurnInDegrees!!
    desiredHeading = robotTurnInDegrees + currHeading #wanted new heading
    print("turn")
    # print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
    # print("heading=" + str(gy.heading()))
    headingError = getError(desiredHeading)
    while (abs(headingError) > 0.3):
        headingError = getError(desiredHeading) #if the robot is turned to the right of the desired heading, -K_P returns a negative value
        brain.screen.print_at(gy.heading(), x=100, y=100)
        brain.screen.print_at(rangeFinderFront.distance(DistanceUnits.IN), x=100, y=200)
        # print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
        # print("heading=" + str(gy.heading()))
        turn(SET_TURN_SPEED, K_P_TURN * headingError)
    stop()
    return robotTurnInDegrees + currHeading
    

      
      
# Program to follow wall, turn left at next wall, and follow wall indefinitely

rangeFinderFront.distance(DistanceUnits.IN)     # acquire initial distance values
rangeFinderRightSide.distance(DistanceUnits.IN)
print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
print("heading=" + str(gy.heading()))
brain.screen.print_at(gy.heading(), x=100, y=100)
# wait(3000)
while (gy.is_calibrating()):
    continue

currHeading = 0
#
print("forward")
currHeading = turnInPlace(SPIN_AROUND_ANGLE, currHeading)
brain.screen.print_at(currHeading, x=200, y=100)
while(rangeFinderFront.distance(DistanceUnits.IN) > SET_DISTANCE_TO_START_FIRST_TURN):
    print('yes' + str(currHeading))
    print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
    print("heading=" + str(gy.heading()))
    brain.screen.print_at(gy.heading(), x=100, y=100)
    brain.screen.print_at(rangeFinderFront.distance(DistanceUnits.IN), x=100, y=200)
    driveAtHeading(currHeading)
stop()
currHeading = turnInPlace(TURN_ANGLE_AT_WALL, currHeading)
print("forward")
while(rangeFinderFront.distance(DistanceUnits.IN) > SET_DISTANCE_TO_START_SECOND_TURN):
    #print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
    #print("heading=" + str(gy.heading()))
    brain.screen.print_at(gy.heading(), x=100, y=100)
    brain.screen.print_at(rangeFinderFront.distance(DistanceUnits.IN), x=100, y=200)
    driveAtHeading(currHeading)
currHeading = turnInPlace(TURN_ANGLE_AT_WALL, currHeading)
print("forward")
while(rangeFinderFront.distance(DistanceUnits.IN) > SET_WALL_DISTANCE):
    #print("dis: " + str(rangeFinderFront.distance(DistanceUnits.IN)))
    #print("heading=" + str(gy.heading()))
    brain.screen.print_at(gy.heading(), x=100, y=100)
    brain.screen.print_at(rangeFinderFront.distance(DistanceUnits.IN), x=100, y=200)
    driveAtHeading(currHeading)
stop()


        
