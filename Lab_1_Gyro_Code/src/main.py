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
K_P_TURN = .1                       #Turning K_P is a multiplier of the drive speed rather than an addition, so the value is far smaller
SET_WALL_FOLLOW_SPEED = 120         # RPM
SET_WALL_DISTANCE = 10            # inches until stops at wall at end
TURN_ANGLE_AT_WALL = -90           # turn left angle in degrees (right = positive)
SET_DISTANCE_TO_START_FIRST_TURN =  10    # inches from wall in front to begin turn
SET_DISTANCE_TO_START_SECOND_TURN = 4     #in
SET_TURN_SPEED = 70                        #RPM
SPIN_AROUND_ANGLE = 180                    #Beginnning spinaround angle DEGREES
currHeading = 0                         #starting heading of the robot

# drive function - For negative values of direction, the robot turns right, and for positive
# values of direction, the robot turns right.  For values of direction with small magnitudes, the
# robot gradually turns.  For values of direction with large magnitudes, the robot turns more
# quickly.

def drive(speed, direction):
   left_motor.set_velocity(speed - direction, RPM)   #direction is a K_P modified value
   right_motor.set_velocity(speed + direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)
#Turns based off a K_P value
def turn(speed, direction):
    left_motor.set_velocity(speed*direction, RPM)
    right_motor.set_velocity(-1 * speed*direction, RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)
#stops the robot
def stop():
    drive(0, 0)

#Finds the error of the heading but corrects for the 0-360 heading jump
def getError(targetHeading):
    headingError = targetHeading - gy.heading()
    #say the heading is 359 degrees and we want to go to 0 degrees, the real difference is 1 degree, but a subtraction results in -359 degrees
    # subtracting that/adding to 360 gives us the true error and corrects for the 360 degree to 0 degree jump
    if (headingError > 180):
        headingError = 360 - headingError
    elif (headingError < -180):
        headingError = 360 + headingError
    return headingError

def driveAtHeading(currHeading):
    headingError = getError(currHeading) #if the robot is turned to the right and the left wheel is too fast (pos)
    #-K_P * error returns a negative error, need to make the right wheel go faster
    drive(SET_WALL_FOLLOW_SPEED, -K_P_DRIVE*headingError)

# Function to turn BaseBot for some number of degrees

def turnInPlace(robotTurnInDegrees, currHeading):
    #turning to the right is positive degrees for robotTurnInDegrees!!
    desiredHeading = robotTurnInDegrees + currHeading #wanted new heading
    print("turn")
    headingError = getError(desiredHeading)
    while (abs(headingError) > 0.3): #while the error is greater than 0.3 degrees then continue to correct using proportional correction
        headingError = getError(desiredHeading) #if the robot is turned to the right of the desired heading, -K_P returns a negative value
        turn(SET_TURN_SPEED, K_P_TURN * headingError)
    stop()
    #stops to reset velocity values 
    return robotTurnInDegrees + currHeading #return value will tell the global variable the new "theoretical" heading
    

      
      
# Program to follow wall, turn left at next wall, and follow wall indefinitely

rangeFinderFront.distance(DistanceUnits.IN)     # acquire initial distance values
rangeFinderRightSide.distance(DistanceUnits.IN)
while (gy.is_calibrating()):
    continue
#waits for the IMU to finish calibrating before the program runs

currHeading = 0 #inital heading
currHeading = turnInPlace(SPIN_AROUND_ANGLE, currHeading)
#spins 180 degrees using proportional control and sets the new heading to 180 degrees

while(rangeFinderFront.distance(DistanceUnits.IN) > SET_DISTANCE_TO_START_FIRST_TURN): #continues going forward until the sensored distance is less than the turn distance
    driveAtHeading(currHeading)
#stops the robot to reset the motor velocities to 0
stop()

#turns 90 degrees to the left using PID, new heading of 90 degrees
currHeading = turnInPlace(TURN_ANGLE_AT_WALL, currHeading)

#drives forward until reaches the second wall using the sensor distance
while(rangeFinderFront.distance(DistanceUnits.IN) > SET_DISTANCE_TO_START_SECOND_TURN):
    driveAtHeading(currHeading)

#turns 90 degrees to the left, new heading of 0 degrees
currHeading = turnInPlace(TURN_ANGLE_AT_WALL, currHeading)

#continues going straight until the wall distance stop (10 in)
while(rangeFinderFront.distance(DistanceUnits.IN) > SET_WALL_DISTANCE):
    driveAtHeading(currHeading)
#stops the robot
stop()


        
