from vex import *

shooterAngle = 0 #zero shooter angle is when the hood is all the way up, positive hood angle is when the hood rotates down
HOOD_SPEED_RATIO = 12/216
K_P_HOOD = 0.01

hood_motor = Motor(Ports.PORT20, 18_1, True)

def resetHood():
    currSpike = False
    while not currSpike:
        currentNow = hood_motor.current()
        hood_motor.spin(FORWARD)
        currentAfter = hood_motor.current()
        if ((currentAfter - currentNow) > 0.5): #runs the motor upwards until it stalls on the hard stops and is detected with a current spike
            currSpike = True
    shooterAngle = 0

def goToHoodAngle(wantedAngle):
    angleDiff = wantedAngle - shooterAngle
    if shooterAngle > wantedAngle:
        hood_motor.spin_to_position(angleDiff * HOOD_SPEED_RATIO, DEGREES)