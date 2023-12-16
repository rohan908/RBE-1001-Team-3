from vex import *
from math import *

shooterAngle = 0 #zero shooter angle is when the hood is all the way up, positive hood angle is when the hood rotates down
HOOD_SPEED_RATIO = 12/216
HOOD_ANGLE_MAX =  #Find exact number this is a place holder ISHAAN!!
MINIMUM_REDCIRCLE_PIXEL_HEIGHT =  #tune this ishaan
DEGREES_PER_PIXEL = #put something here ishaan
RED_CIRCLE_DIAM_PIXELS = #red circle outside diameter ini pixels
GREEN_YELLOW_TAPE_WIDTH_PIXELS = #green yellow tape width in pixels

HOME_HOOD_ANGLE = 0
MOVED_HOOD_ANGLE = 1
CURR_HOOD_STATE = HOME_HOOD_ANGLE
#Use state machine to reset hood after every shot

hood_motor = Motor(Ports.PORT20, 18_1, True)
shooter_cam = Vision(Ports.PORT13, 50, SIG_YELLOWGREEN_TAPE, SIG_REDCIRCLE)

REDdisVsHoodAngle = [[0,0], [dis, angle], [dis, HOOD_ANGLE_MAX]] #ordered lists where distance values increase per input
BLUEdisVsHoodAngle = [[0,redHoodAngleStart],[dis, angle], [dis, HOOD_ANGLE_MAX]]



def resetHood():
    #Uses current spiking to determine when the hard limit is reached
    currSpike = False
    while not currSpike:
        currentNow = hood_motor.current()
        hood_motor.spin(FORWARD)
        currentAfter = hood_motor.current()
        if ((currentAfter - currentNow) > 0.5): #runs the motor upwards until it stalls on the hard stops and is detected with a current spike
            currSpike = True
    return 0

def goToHoodAngle(wantedAngle):
    angleDiff = wantedAngle - shooterAngle
    hood_motor.spin_to_position(angleDiff / HOOD_SPEED_RATIO, DEGREES)

def interpolateForHoodAngle(disVsAngleList, wantedDis):
    #inputs a 2xN array where each sublist of N is distance, hood angle
    #interpolates for a given distance to find a new hood angle for accurate shooting
    #distance is x var, hood angle is y var
    x1 = disVsAngleList[0][0]
    y1 = disVsAngleList[0][1]
    x2 = disVsAngleList[0][0]
    y2 = disVsAngleList[0][1]
    for i in range(1, len(disVsAngleList)):
        if disVsAngleList[i-1][0] <= wantedDis:
            x1 = disVsAngleList[i-1][0]
            y1 = disVsAngleList[i-1][1]
            x2 = disVsAngleList[i][0]
            y2 = disVsAngleList[i][1]
    m = (y2 - y1) / (x2 - x1)
    # y - y1 = m(x-x1) => y = mx - mx1 + y1
    hoodFinal = m*wantedDis - m*x1 + y1
    return hoodFinal

def ReturnRedCircle():
    objects = shooter_cam.take_snapshot(SIG_REDCIRCLE, 10)
    if (objects):
        print(" x: ", shooter_cam.largest_object().centerX, "y: ", shooter_cam.largest_object().centerY, "width", shooter_cam.largest_object().width)
        listOfPossibleRedCircle = []
        minSize = 0
        largestObjectIndex = 0
        currIndex = 0
        for i in objects:
            if i.centerY < MINIMUM_REDCIRCLE_PIXEL_HEIGHT:
                #camera is inverted, it really checks if it is ABOVE the minimum height
                #makes sure we do not detect red balls
                listOfPossibleRedCircle.append(i)
                if i.width * i.height > minSize:
                    largestObjectIndex = currIndex
                currIndex = currIndex + 1
        return (True, listOfPossibleRedCircle[largestObjectIndex])
    return (False, None)
            
def RedCircleDistance():
    foundRedCircle, redCircleObject = ReturnRedCircle()
    if foundRedCircle:
        return (RED_CIRCLE_DIAM_PIXELS) / atan(redCircleObject.width * DEGREES_PER_PIXEL)
    else: 
        return -1
    
def ReturnBlueCircle():
    objects = shooter_cam.take_snapshot(SIG_YELLOWGREEN_TAPE, 1)
    if (objects):
        print(" x: ", shooter_cam.largest_object().centerX, "y: ", shooter_cam.largest_object().centerY, "width", shooter_cam.largest_object().width)
        return (True, objects)
    return (False, None)

def BlueCircleDistance():
    detectedBlueCircle, greenYellowTape = ReturnBlueCircle()
    if detectedBlueCircle:
        return (GREEN_YELLOW_TAPE_WIDTH_PIXELS) / atan(greenYellowTape.width * DEGREES_PER_PIXEL)
    else: 
        return -1

        

def changeHoodAngle(ballIsRed, currDistance):
    #moves the hood angle depending on distance to the wall and the ball color
    if ballIsRed:
        hoodAngle = interpolateForHoodAngle(REDdisVsHoodAngle, currDistance)
        goToHoodAngle(hoodAngle)
    else: 
        hoodAngle = interpolateForHoodAngle(BLUEdisVsHoodAngle, currDistance)
        goToHoodAngle(hoodAngle)

def configHood(ballIsRed):
    #use this function in the state machine, brings all the above functions together
    #Use the RESET HOOD function after every call to ensure that the shooter is most accurate
    if ballIsRed:
        dis = RedCircleDistance()
        if dis != -1:
            changeHoodAngle(True, dis)
        else: 
            print("Red circle not found")
    else:
        dis = BlueCircleDistance
        if dis != -1:
            changeHoodAngle(False, dis)
        else:
            print("Blue circle not found")

    
    
    