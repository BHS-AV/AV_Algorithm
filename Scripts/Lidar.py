import Controls
import rospy
import numpy as np
import Navigation as nav
import Map as m
import time

reversing = 0
maxSpeed = 6
lastTime = 0
lt=0
destDir = 1 # 0 = north , 90 = east , 180 = south , 270 = west
hold=0 # 0 = normal , 1 = right , -1 = left
lastPathFind=time.time()
lastturn=0
turn=0
turnStart=0
navMode=2 # 0 = standard ; 1 = reversing why not work
orient=0
dataString=""
speed=0
x=0
prefdir=0

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

def setUTurn():
    global navMode
    navMode=1

def print_data(data):
    # GLOBAL VARIABLES
    global reversing
    global maxSpeed
    global hold,lt,lastPathFind,lastturn,turn,navMode,turnStart,orient,dataString,speed,x,prefdir

    # TIME
    global lastTime
    if (lastTime == 0):
        lastTime = rospy.get_time()
    thisTime = rospy.get_time()
    dt = thisTime - lastTime
    lastTime = thisTime

    # DATA ARRAYS
    fulldata=get_data_array(data.ranges,0,240)
    dataFront = get_data_array(data.ranges, 100, 140)
    dataLeft = get_data_array(data.ranges, 120, 220)
    dataRight = get_data_array(data.ranges, 20, 120)

    # DISTANCES
    distFront = dataFront.mean()
    distLeft = dataLeft.mean()
    distRight = dataRight.mean()

    #MAPPING
    m.update(nav.getOrient())
    if (rospy.get_time() > lt + .2 or (isBreaking() and rospy.get_time() > lt + .15)):
        mapping=not isBreaking()
        lt = rospy.get_time()
        m.scanWalls(data.ranges,distLeft,distRight,distFront,dataString, mapping)

    # ORIENTATION
    orient = nav.getOrient()
    # Basic navigation

    if navMode==0:  # STANDARD CONTROLS
        if(m.route!=None):
            #TODO FIX THIS
            x=1
            speed=3
            turn=-m.route.dif/3.15
        else:
            if(reversing==0):
                standardControls(distRight,distLeft,dataFront,distFront)
            else:
                reverse(reversing,distFront,distLeft,distRight)
    elif navMode==1:
        speed=0
        turn=0
        x=1
    elif navMode==2:
        speed=0
        turn=0
        x=0
        rows=findMaxes(fulldata)
        '''
        Plan
        
        1 - find a max
        2 - go that direction
        3 - if can no longer go in that direction,  restart
        '''
        if(len(rows)!=0):
            best=rows[0]
            br = ((  best[len(best) - 1] / 4.5)-(best[0] / 4.5))/((abs((best[len(best)/2]/4.5)-120)/120))#this score is bugged
            for r in rows:
                rg=((  r[len(r) - 1]) / 4.5-(r[0] / 4.5))/(pow(abs((r[len(r)/2]/4.5)-120)/120,2))
                print ("row from "+str((r[len(r)-1]/4.5))+" - "+str((r[0]/4.5))+" is "+str(rg))
                if(rg>br):
                    br=rg
                    best=r
            destd=(best[0]/2.0)+(((  best[len(best) - 1] / 4.5)-(best[0] / 4.5))/2.0)-120
            prefdir=destDir-120+orient
            if(prefdir<0):prefdir=prefdir+360
            print("optimal dir is at "+str(round(destd))+" ("+str(round(best[0]/4.5-120))+"-"+str(round(best[len(best)-1]/4.5)-90)+")")
            navMode=3
            #speed=1
            #x=1
            #turn=(destd/60) if abs(destd)<60 else (-1 if turn<0 else 1)
        elif(dataFront.mean()>6 or fulldata.mean()>8):
            print("no optimal dir")
            speed=2
            x=1
    elif navMode==3:
        dor=prefdir-orient
        if(abs(dor)>180):
            dor=prefdir+360-orient
            if (abs(dor) > 180):
                dor=prefdir-360-orient
        turn=dor/180
        print ("turning "+str(round(turn,3))+" from "+str(round(orient,2))+" to "+str(round(prefdir,2)))
        turn = turn if abs(turn)<1 else (1 if turn>0 else -1)
        speed=(1-abs(turn))*2
        x=1
        if( dataFront.mean()<.5):
            navMode=2
            speed=0
            x=0
            turn=0

    dataString=" o : "+str(round(orient))+" | navmode "+str(navMode)+" | dist  ("+str(round(distLeft,1))+" "+str(round(distFront,1))+" "+str(round(distRight,1))+") | turn "+str(round(turn,1))+" | speed "+str(round(speed))

    if (isBreaking()):
        if (time.time() - lastPathFind > 4):
            lastPathFind = time.time()
            print("finding ppaths")
            m.findPPaths()
    #turn=(turn*.3)+(lastturn*.7)
    #lastturn=turn
    Controls.move(x, turn, speed,dt)

def getOrientDif(o1,o2):
    '''```o1 = np.rad2deg(self.orient) + 90
o2 = np.rad2deg(nfunc.orient) + 90
if (o1 > o2):
    o1 = np.rad2deg(nfunc.orient) + 90
    o2 = np.rad2deg(self.orient) + 90
    dor = abs(o1 - o2)
    dor1 = abs((o1 + 180) - o2)
    if (dor < dor1):
        return dor
    return dor1```'''

def findMaxes(data):
    mean=data.mean()
    ppd=4.5
    p1=np.percentile(data,75)
    highest=[]
    rows=[]
    r=None
    for i in range(len(data)):
        if(data[i]>p1):
            if(r!=None):
                r.append(i)
            else:
                r=[i]
        else:
            if(r!=None):
                if(len(r)/4.5>2):
                    rows.append(r)
                    r=None
            #highest.append(i)
    for r1 in rows:
        min=r1[0]/4.5
        max=r1[len(r1)-1]/4.5
        print("row  "+str(round(min))+" - "+str(round(max)))
    #print p1
    return rows

def reverse(dir,df,dl,dr):
    global turn,lastPathFind,reversing,speed,x,maxSpeed
    if(df>4):
        reversing=0
    else:
        speed=1
        x=-1
        turn=1

def getAngDif(a1,a2):
    adif = a2 - a1
    if(adif<-3.14159):
        adif=adif+(3.14159*2)
    if(adif>3.14159):
        adif=adif=adif-(3.14159*2)
    if (abs(adif) > 3.14159):
        adif = a2 + (3.14159 * 2) - a1
    if (abs(adif) > 3.14159):
        adif = a2 - (3.14159 * 4) - a1
    return adif

def turnAround():
    global turn,orient,turnStart,navMode
    turn=1
    if(abs(turnStart-orient)-180<10 or abs(turnStart-orient-360)-180<10 or abs(turnStart-orient+360)<10):
        navMode=0

def standardControls(distRight,distLeft,dataFront,distFront):
    global turn,lastPathFind,reversing,speed,x,maxSpeed

    x = 1
    turn = (distRight / (distLeft + distRight)) * 2.0 - 1

    turn = -turn
    speed=(maxSpeed*(1-abs(turn)))#TODO THIS
    speed= speed*((distFront/6) if distFront<6 else 1)
    turn = limitTurn(turn)

    if (abs(distRight - distLeft) < (distRight + distLeft) / 8):
        turn = turn * ((maxSpeed - speed) / maxSpeed)


    if (needsToReverse(dataFront, distFront) == 1):
        #stop()
        reversing = 1
        if (distLeft > distRight):
            reversing = -reversing

def stop():
    global navMode
    print ("BREAKING")
    navMode=1

def isBreaking():
    global navMode
    return navMode==1

def limitTurn(turn):
    if(abs(turn)<1):
        return turn
    else:
        return 1 if turn>0 else -1

def isGoingStraight():
    return abs(lastturn)<.1

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

