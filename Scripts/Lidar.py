import Controls
import rospy
import numpy as np
import Navigation as nav
import Map as m
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import LaserScan

reversing = 0
maxSpeed = 3
lastTime = 0
lt=0
destDir = 1 # 0 = north , 90 = east , 180 = south , 270 = west
hold=0 # 0 = normal , 1 = right , -1 = left


def get_data_array(data, a=0, b=240):
    PPD = 4.5  # points per degree
    data = data[int(a * PPD):int(b * PPD)]
    arr = np.array(data)
    return arr


def needsToReverse(dataFront, distFront, range=1):
    if (distFront < range):  # '''or dataFront.min() < range/2.0'''
        return 1
    return 0

def getOrient():
    return nav.getOrient()

def print_data(data):
    # GLOBAL VARIABLES
    global reversing
    global maxSpeed
    global hold,lt

    if (rospy.get_time()>lt+.8):
        lt=rospy.get_time()
        m.scanWalls(data.ranges)

    # TIME
    global lastTime
    if (lastTime == 0):
        lastTime = rospy.get_time()
    thisTime = rospy.get_time()
    dt = thisTime - lastTime
    lastTime = thisTime

    # DATA ARRAYS
    dataFront = get_data_array(data.ranges, 100, 140)
    dataLeft = get_data_array(data.ranges, 120, 220)
    dataRight = get_data_array(data.ranges, 20, 120)

    # DISTANCES
    distFront = dataFront.mean()
    distLeft = dataLeft.mean()
    distRight = dataRight.mean()

    # ORIENTATION
    orient = nav.getOrient()
    relDestDir = destDir - orient

    # WALL ANGLES
    #rAng = getRelativeWallsOrient(data, 30, 60)
    #lAng = getRelativeWallsOrient(data, 210, 240)
    a1=20
    a2=40
    rAng = getRelativeWallsOrient(data, a1, a2, 1)-60-a1
    lAng =  getRelativeWallsOrient(data, -a2, -a1, 0)-60-a1
    dAng = rAng-lAng
    straightness = (rAng + lAng)

    # PATHS
    rOpenings = scanOpenings(data, 5, 115)
    lOpenings = scanOpenings(data, 155, 265)

    # DEFAULT MOVEMENT
    x = 1
    turn = (distRight / (distLeft + distRight)) * 2.0 - 1
    #if (straightness<20):
    #    turn = (turn + ((rAng+lAng)/2))/2

    turn = -turn
    speed = limit_speed(((((distRight + distLeft) / 4.0) + distFront) / 1.25) * maxSpeed, maxSpeed)




    ttt=0
    if (hold!=0 and straightness>5):
        #print (len(rOpenings)," openings to the right, and ", len(lOpenings)," to the left")
        if hold==1:
            '''if (isOpeningBetween(data, 45, 135)):
                turn=distRight/2
                if (turn>1):
                    turn=1
                turn=-turn
                print("holding right")
                ttt=1'''
            turn=rAng/30
            ttt=1
        else:
            turn = lAng / 30
            ttt=1

    if (turn > 1):
        turn = 1
    elif (turn < -1):
        turn = -1

    '''
    if (abs(turn)>.3):
        speed = speed * (1.2 - abs(turn))
    '''
    '''
    if (straightness > 25):
        speed /= 1.5
    elif (straightness < 1):
        speed *= 1.5
    '''

    xorient = nav.getXOrient() * 180 / 3.14
    yorient = nav.getYOrient() * 180 / 3.14

    if reversing == 0:  # STANDARD CONTROLS

        # BASIC LOW-LEVEL TURNING
        if (abs(distRight - distLeft) < (distRight + distLeft) / 4 and ttt==0):
            turn = turn * ((maxSpeed - speed) / maxSpeed)

        # CRASH IMINENT OVERRIDE
        if (needsToReverse(dataFront, distFront) > 0):
            reversing = 1
            if (distLeft > distRight):
                reversing = -reversing

    elif reversing != 0:  # REVERSING

        # CHECK TO SEE IF RUNNING INTO SOMETHING
        if xorient > 10 or yorient > 10:
            reversing = 0

        # BASIC REVERSING
        turn = reversing
        x = -1
        speed = speed / 4

        # CHECKS TO SEE IF STILL NEED TO TURN
        if (needsToReverse(dataFront, distFront, 2) == 0):
            reversing = 0

    '''
    #m.update(nav.getOrient)
    print ("")
    print ("openings to right: ",rOpenings,".")
    print ("wall angle right = ", int(round(rAng)))
    print ("wall angle left = ", int(round(lAng)))
    print ("orient = ", int(round(360-orient)), " (",int(round(orient)),")")
    print ("straighness = ", (int(round(straightness*100))/100.0))
    print ("reversing = ", reversing)
    print ("turning ",(int(round(turn*100))/100.0), " and moving ", x, " at a speed of ", (int(round(speed*100))/100.0))
    print ("left ", (int(round(distLeft*100))/100.0), " forward ", (int(round(distFront*100))/100.0), " right ", (int(round(distRight*100))/100.0))
    #print ("dtime =", dt)
    print ("")'''

    Controls.move(x, turn, speed)

def getDistAt(data, ang1):
    print( "dist at angle", ang1," = ", get_data_array(data.ranges, ang1-1, ang1+1).mean())

def isOpeningBetween(data, ang1, ang2):
    allData=get_data_array(data.ranges, 110, 160)
    dataHere=get_data_array(data.ranges, ang1, ang2)
    if (dataHere.max()>allData.mean()*.75):
        return 1
    return 0

def scanOpenings(data, ang1, ang2):
    startAng1=0
    if ang1<235-ang2:
        startAng1=1
    lastWall=0
    openingDetected = []
    recordedLast=0
    for i in range((ang2 - ang1) / 2):
        nang1=0
        nang2=0
        if startAng1==1:
            nang1 = ang2 - ((i+1) * 2)
            nang2 = ang2 - (i * 2)
        else:
            nang1 = ang1 + (i * 2)
            nang2 = ang1 + ((i + 1) * 2)
        datahere =get_data_array(data.ranges, nang1, nang2)
        wallhere = datahere.mean()
        if (recordedLast==1):
            if (abs(abs(wallhere) - abs(lastWall)) > 1.5):
                if(abs(((nang1+nang2)/2)-135)>40):
                    openingDetected.append([nang1, nang2])
                    #print("opening at ",nang1 ," - ", nang2)

        if (datahere.mean()>8):
            recordedLast=0
        else:
            lastWall=wallhere
            recordedLast=1
    return openingDetected

def getRelativeWallsOrient(data, ang1, ang2, scanningRight):
    C = abs(ang2 - ang1)
    a = get_data_array(data.ranges, ang1 - 1, ang1 + 1).mean()
    b = get_data_array(data.ranges, ang2 - 1, ang2 + 1).mean()
    #print("a= ",a," b= ", b)
    orient=0
    C = C * 3.14 / 180.0
    if (scanningRight==1):
        orient = getOrientBetweenPoints(C, a, b)
    else:
        orient = getOrientBetweenPoints(C, b, a)
    orient = orient * 180.0 / 3.14

    return orient


def getOrientBetweenPoints(C, a, b):

    c = np.math.sqrt((a * a) + (b * b) - (2 * a * b * np.math.cos(C)))

    A = np.math.asin(a / (c / np.math.sin(C)))

    return A


def limit_speed(speed, limit=maxSpeed):
    if (speed > limit):
        speed = limit
    return speed

