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

#Instantiating Devices
brain = Brain()
controller = Controller()

gyro = Inertial(Ports.PORT6)
gyro.reset_heading()
gyro.calibrate()

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

# def auton_5ballcollect():
#     #turn on intake
#     for i in range(5):
#         while rangeFinder.distance(DistanceUnits.IN):
            
#         #move fowards to collect the ball
#             #while robot is < x inches from the back wall
#                 #move forward at the current heading
#         #back up
#             #while robot is > x inches from the back wall
#                 #move backwards at the current heading
#         #wait
#             #wait for 2 seconds

def DetectObject():
    objects = indexer_camera.take_snapshot(SIG_RED_BALL)
    if (objects):
        print(" x: ", indexer_camera.largest_object().centerX, "y: ", indexer_camera.largest_object().centerY, "width", indexer_camera.largest_object().width)
        return True
    else:
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
    brain.screen.print_at(shooter_motor.velocity(RPM), x=100, y=200)
    #Indexer Pseudocode
    #If the robot detects a ball, move indexer by x amount, stops otherwise.
    # red_objects = indexer_camera.take_snapshot(SIG_RED_BALL)
    # blue_objects = indexer_camera.take_snapshot(SIG_BLUE_BALL)

    # if(DetectObject()):
    #      print('height:', indexer_camera.largest_object().height, '   width:',  indexer_camera.largest_object().width)
    #      wait(200)
    #      indexer_motor.spin_for(FORWARD, 100, DEGREES)
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
        indexer_motor.spin(FORWARD, 200, RPM)
    if controller.buttonA.pressing():
        shooter_motor.spin(FORWARD, 200, RPM)
    elif controller.buttonB.pressing():
        intake_motor.spin(FORWARD, 0, RPM)
        indexer_motor.spin(FORWARD, 0, RPM)
        shooter_motor.spin(FORWARD, 0, RPM)
    if controller.buttonX.pressing():
        hood_motor.spin(FORWARD, 0, RPM)

    if controller.buttonR1.pressing():
        hood_motor.spin(FORWARD, 10, RPM)
    elif controller.buttonR2.pressing():
        hood_motor.spin(REVERSE, 10, RPM)
        




        
