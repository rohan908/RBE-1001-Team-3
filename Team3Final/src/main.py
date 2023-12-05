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

#Instantiating motors
frontLeft_motor = Motor(Ports.PORT12, 18_1, True)
frontRight_motor = Motor(Ports.PORT13, 18_1, False)
backLeft_motor = Motor(Ports.PORT11, 18_1, True)
backRight_motor = Motor(Ports.PORT20, 18_1, False)
intake_motor = Motor(Ports.PORT15, 18_1, True)

while True:
    frontLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    frontRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    backLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    backRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    frontLeft_motor.spin(FORWARD)
    frontRight_motor.spin(FORWARD)
    backLeft_motor.spin(FORWARD)
    backRight_motor.spin(FORWARD)
    if controller.buttonL1.pressing():
        intake_motor.spin(FORWARD, 200, RPM)
    elif controller.buttonL2.pressing():
        intake_motor.spin(REVERSE, 200, RPM)
    elif controller.buttonB.pressing():
        intake_motor.spin(FORWARD, 0, RPM)
        




        
