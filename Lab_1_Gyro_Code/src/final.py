# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
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
frontLeft_motor = Motor(Ports.PORT10, 18_1, True)
frontRight_motor = Motor(Ports.PORT1, 18_1, False)
backLeft_motor = Motor(Ports.PORT10, 18_1, True)
backRight_motor = Motor(Ports.PORT10, 18_1, True)

while True:
    frontLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    frontRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)
    backLeft_motor.set_velocity(2* (controller.axis3.position()+controller.axis1.position()), RPM)
    backRight_motor.set_velocity(2* (controller.axis3.position()-controller.axis1.position()), RPM)




        
