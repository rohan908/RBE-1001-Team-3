# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       lineTracker.py                                                      #
# 	Author:       Ishaan                                                       #
# 	Created:      10/25/2023, 1:29:32 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

#Robot Constants
WHEEL_DIAMETER = 4
ROBOT_TRACK = 11
GEAR_RATIO = 5

#PID controllers
KP_STRAIGHT = 0.1 #PID controller
KP_TURN = 0.1 #PID controller

#Other Constants
TARGET_DISTANCE = 10 #Inches to the wall

# Brain should be defined by default
brain=Brain()

#Instantiating motors
left_motor = Motor(Ports.PORT10, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)
arm_motor = Motor(Ports.PORT8, 18_1, False)

#Instantiating sensors
rangeFinderFront = Sonar(brain.three_wire_port.g)
rangeFinderRightSide = Sonar(brain.three_wire_port.a)

leftLineSensor = Line(brain.three_wire_port.c)
rightLineSensor = Line(brain.three_wire_port.d)

# turnDegrees : Integer or Float -> None
# Consumes a number (in degrees) that the robot will turn
# RESULT: The robot will rotate "angle" degrees
# A positive number is counter-clockwise, a negative number is clockwise
def turnDegrees(angle):
    left_motor.spin_for(REVERSE, angle*(float(ROBOT_TRACK/WHEEL_DIAMETER))*GEAR_RATIO, DEGREES, 150, RPM, False)        
    right_motor.spin_for(FORWARD, angle*(float(ROBOT_TRACK/WHEEL_DIAMETER))*GEAR_RATIO, DEGREES, 150, RPM, True)

# turnUntil White : None -> None
# RESULT : The robot will keep turning counter clockwise until both line trackers have reached a white line
def turnUntilWhite():
    print(leftLineSensor.value()) # getting an initial reading
    print(rightLineSensor.value())
    wait(250) #delay for the robot to calculate the above lines
    while leftLineSensor.value() > 1500 and rightLineSensor.value() > 1500: #the value of a white line is <1500
        left_motor.set_velocity(-100, RPM) 
        right_motor.set_velocity(100, RPM)
        left_motor.spin(FORWARD)
        right_motor.spin(FORWARD)
    
# drivePID : Integer or Float, Integer or Float -> None
# RESULT : The robot will set the velocity at the given speed, with adjustments by the error parameter
#          A positive error will turn the robot counter-clockwise, and a negative error will turn the robot clockwise
def drivePID(speed, error):
    left_motor.set_velocity(speed - error, RPM)
    right_motor.set_velocity(speed + error, RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)

# followLine : Integer or Float -> None
# RESULT : Until the robot is "stopDist" inches away from the wall (based on the sensor mounted at the front),
#          it will move forwards, following the white line
def followLine(stopDist):
    while rangeFinderFront.distance(DistanceUnits.IN) > stopDist: # Stays in the loop until the ultrasonic sensor is "stopDist" inches away from the wall
        drivePID(100, KP_STRAIGHT * (leftLineSensor.value() - rightLineSensor.value())) #Sets motor speed using a PID
    drivePID(0, 0) #Sets wheel motor speed to 0 once the robot is at the target range

def followPath():
    rangeFinderFront.distance(DistanceUnits.IN) #Gets an initial distance from the wall ahead of the front ultrasonic sensor
    wait(250) #delay for robot to calculate the above line
    turnDegrees(-190) #turns the robot 180 degrees
    followLine(12.5) #follow the line until 12.5 inches from the wall
    turnDegrees(20) #turns the robot 20 degrees counter clockwise
    turnUntilWhite() #keeps turning the robot until the white line is detected
    followLine(5) #follow the line until 5 inches from the wall
    turnDegrees(20) #turns the robot 20 degrees counter clockwise
    turnUntilWhite() #keeps turning the robot until the white line is detected
    followLine(TARGET_DISTANCE) #follow the line until 10 inches from the wall
    
followPath()
