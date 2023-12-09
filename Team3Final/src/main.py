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

#Instantiating motors
frontLeft_motor = Motor(Ports.PORT11, 18_1, True)
frontRight_motor = Motor(Ports.PORT20, 18_1, False)
backLeft_motor = Motor(Ports.PORT12, 18_1, True)
backRight_motor = Motor(Ports.PORT19, 18_1, False)
intake_motor = Motor(Ports.PORT10, 18_1, True)
indexer_motor = Motor(Ports.PORT1, 18_1, True)

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


while True:
    frontLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    frontRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    backLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    backRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    frontLeft_motor.spin(FORWARD)
    frontRight_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)
    
    if controller.buttonL2.pressing():
        intake_motor.spin(FORWARD, 200, RPM)
        indexer_motor.spin(REVERSE, 200, RPM)
    elif controller.buttonL1.pressing():
        intake_motor.spin(REVERSE, 200, RPM)
        indexer_motor.spin(FORWARD, 200, RPM)
    elif controller.buttonB.pressing():
        intake_motor.spin(FORWARD, 0, RPM)
        indexer_motor.spin(FORWARD, 0, RPM)
    # if controller.buttonR1.pressing():
    #     indexer_motor.spin(FORWARD, 200, RPM)
    # elif controller.buttonR2.pressing():
    #     indexer_motor.spin(REVERSE, 200, RPM)
    # elif controller.buttonY.pressing():
    #     indexer_motor.spin(FORWARD, 0, RPM)
        




        
