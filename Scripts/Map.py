import numpy as np
import itertools
import Lidar as lid
from graphics import *

height=1000
width=1000
anchorpoint=Point(width/2,height/2)


win = GraphWin("map_display_window", width, height)
image=Image(anchorpoint, height, width)

c=None
front=None
scale=4
x=win.getWidth()/2*scale
y=win.getHeight()/2*scale
orient=0
lt=time.time()
lt2=time.time()-.25
lscale=80.0
scantime=0
refbool=0

oldLocs=[Point(x / scale, y / scale)]
points=[]
#points=[]
allwalls=[]
wall=[]
nodes=[]
oldNodes=[]
carpath=None
ppaths=[]
connections=[]
similarpos=[]
fov=3.14159*4.0/3.0

subtimes=[0,0,0,0,0,0,0]
#                       WHAT ARE THE SUBTIMES?
# 0 - scanning , 1 - cleaning data , 2 - connecting
# 3 - cleaning , 4  - routing      , 5 - locating exits, 6 - archival
methodIterations=[0,0,0]
# 0 - scan/update , 1 - cleaning methods , 2 - locating exits

lastCorrection=time.time()
haslapped=0
nearbyNodes=[]
nearbyWalls=[]
routedirs=[]
dirIntersects=[]
lastCarState=None

goal=None
timeOfLastLoop=0
loops=0
shouldUTurn=False

def update(dir):
    global win,front,c,x,y,orient, oldLocs
    d1=dir/180.0*3.14
    orient=d1

def render(dt):
    global win,front,c,x,y,orient,oldNodes, routedirs,oldLocs,points,allwalls,dirIntersects,scantime,wall,carpath,wallpairs,lastCarState,ppaths,similarpos,methodIterations
    cx=x
    cy=y
    co=orient
    if(lastCarState!=None):
        cx=lastCarState.x
        cy=lastCarState.y
        co=lastCarState.orient
    crp=Point(cx/scale,cy/scale)

    c = Circle(Point(cx/scale,cy/scale), 7)
    c.setFill("black")
    front = Circle(Point(cx / scale + (7 * np.math.cos(co)), cy / scale + (7 * np.math.sin(co))), 5)
    front.setFill("red")

    s2='last scan time : '+str(scantime)+' s, render time = '+str(round(dt,2))
    t2loc=Point(500,15)
    t2=Text(t2loc,s2)
    t2.setSize(11)

    s3=getTotalNodeData()
    t3loc=Point(500,45)
    t3=Text(t3loc,s3)
    t3.setSize(11)

    s4 = 'method iterations : '+str(methodIterations)
    if(carpath!=None):
        s4 = s4+' , recorded locations : '+str(len(carpath.path))+" , loops : "+str(loops)
    t4 = Text(Point(500, 75), s4)
    t4.setSize(11)


    clear(win)

    if carpath!=None:
        dirLines = carpath.getOrientLines()
        path=carpath.getLines()
        for l in path:
            l.setFill("grey")
            l.draw(win)
        for l in dirLines:
            l.setFill("blue")
            l.draw(win)

    for n in nodes:
        size=len(n.cn)
        #rad=size/2.0+1
        rad=2
        p2 = Circle(n.p, rad)
        if size<6 and size>2:
            cmult=size/5.0
            p2.setFill(color_rgb(250*cmult,0,0))#250*size,0,250*(1-size)
        elif size==1:
            #p2=Circle(n.p,4)
            p2.setFill("orange")
        elif size==0:
            p2=Circle(n.p,2)
            p2.setFill("yellow")
        elif size==2:
            p2=Circle(n.p,2)
            p2.setFill("white")
        else:
            p2.setFill("red")
        p2.draw(win)
        lines=n.getLines()
        for l in lines:
            l.draw(win)

    for n in oldNodes:
        size=len(n.cn)
        #rad=size/2.0+1
        p2 = Circle(n.p, 1)
        p2.setFill('grey')
        p2.draw(win)
        lines=n.getLines()
        for l in lines:
            l.draw(win)

    for p in ppaths:
        c1=Line(p[0].p,p[1].p)
        c1.setFill("orange")
        c1.draw(win)

    for p in similarpos:
        p1=Circle(p.p,5)
        p1.setFill("green")
        p1.draw(win)

    for dirdata in routedirs:
        dir=dirdata.dir
        confidence=dirdata.instances
        confidenceMult=(confidence/3.0)
        ldx=(40 * np.math.cos(dir) * confidenceMult)
        ldy=(40 * np.math.sin(dir) * confidenceMult)
        linep1=Point(crp.x + ldx , crp.y + ldy )
        linep2=Point(crp.x - ldx , crp.y - ldy )
        line=Line(linep1,linep2)
        line.setArrow('both')
        line.setFill("green")
        line.draw(win)
        '''lineText=Text(linep1,dirdata.toString())
        lineText.setFill('green')
        lineText.setSize(6)
        lineText.draw(win)'''
    '''for w in nearbyWalls:
        wline=Line(w.n1.p,w.n2.p)
        wline.setFill('red')
        wline.draw(win)'''
    for ip in dirIntersects:
        circ=Circle(ip,5)
        circ.setFill('red')
        circ.draw(win)

    t2.draw(win)
    t3.draw(win)
    t4.draw(win)

    c.draw(win)
    front.draw(win)

def scanWalls(data,dl,dr,df, datastring):
    global orient,x,y,lt,oldLocs,points, scantime,allwalls,wall,nodes, scale, lscale,lastCarState, nodes, refbool, lastCorrection, haslapped
    if (orient==0):return
    samples=30
    st=time.time()
    scanrange=50
    prange=len(nodes)*.8
    if prange>scanrange:scanrange=prange

    if(haslapped):
        samples=round(samples/3)

    for i in range(samples):
        lp1=getPoint(data,i*(115/samples)+5)
        rp1=getPoint(data,240-(i*(115/samples)+5))
        if (lp1!=None):
            points.append(lp1)
        if (rp1!=None):
            points.append(rp1)

    subtimes[0]=round(time.time()-st,3)
    st=time.time()


    mininitdist=.37*lscale/scale
    maxCombineDist=1*lscale/scale
    minCombineDist=.2*lscale/scale
    addedNodes=[]
    for p in points:
        n=Node(p)
        n1=getClosestNode(n,40)
        if(n1!=None):
            n1nd=n.distToNode(n1)
            if (n1nd>mininitdist):
                nodes.append(n)
                addedNodes.append(n)

        else:
            nodes.append(n)
            addedNodes.append(n)
            #numadded=numadded+1

    for n in addedNodes:
        n1=getClosestNode(n,40)
        if(n1!=None):
            n1nd=n.distToNode(n)
            #if(n1nd)
    #for n in nodes:

    #findPPaths()
    findRoutes()
    if(len(routedirs)>1):
        print("multiple routes")
        #lid.stop()
    datastring=datastring," | routes : "

    for dirdata in routedirs:
        datastring=datastring, round(dirdata.dir,2)
    print (datastring)
    points=[]
    subtimes[1]=round(time.time()-st,3)
    nodes = connectNodes(nodes)

    if (time.time() > lt + .5):
        updatePath(Point(x / scale, y / scale),dl,dr,df)
        if(time.time()-lastCorrection>10):
            getSimilarPos(dl,dr,df)
        cleanNodes(scanrange)
        updateCarState()
        #findRoutes()
        hasLooped()
        methodIterations[1]=methodIterations[1]+1

    methodIterations[0]=methodIterations[0]+1
    scantime = getSubTimes(subtimes)


def updateCarState():
    global lastCarState,x,y,orient
    if lastCarState==None:
        lastCarState=carState(x,y,orient)
    else:
        lastCarState.update(x,y,orient)


def shouldUTurnNow():
    global shouldUTurn
    if shouldUTurn:
        shouldUTurn=False
        return True
    return False

def hasLooped():
    global carpath,methodIterations,x,y,scale,lscale, timeOfLastLoop,loops,shouldUTurn
    car=Point(x/scale,y/scale)
    maxdist=2.0*lscale/scale
    if(time.time()-timeOfLastLoop>5):
        firstPos=carpath.path[0]
        dx=car.x-firstPos.p.x
        dy=car.y-firstPos.p.y
        dist=np.math.sqrt(dx*dx+dy*dy)
        #print 'start = ',firstPos.p,' now = ',car
        #print 'dist = '+str(dist)+" of "+str(maxdist)
        if(maxdist>dist):
            timeOfLastLoop = time.time()
            #lid.setUTurn()
            loops += 1
            if(loops==1):
                shouldUTurn=True


def findRoutes():
    global nodes, orient, x,y,scale,lscale,subtimes,methodIterations,routedirs,nearbyWalls,nearbyNodes
    st=time.time()
    nearbyNodes=[]
    car=Point(x/scale,y/scale)
    maxdist=5.0*lscale/scale

    fov = 3.14159 * 4 / 3
    right = orient + (fov / 2)
    left = orient + (fov / 2)
    tau=3.14159*2

    #st1=time.time()

    for n in nodes:
        dx=n.p.x-car.x
        if(abs(dx)<maxdist):
            dy = n.p.y - car.y
            if(abs(dy)<maxdist):
                pass
                nor=np.math.tan(dy/dx)
                if(dx<0):
                    nor=nor+3.14159
                dori=nor-orient
                if(abs(dori)<fov/2 or abs(dori-tau)<fov/2 or abs(dori+tau)<fov/2):
                    i=nodes.index(n)
                    #print('for node ',i,' dif between ur orient ', round(orient,2), ' and relative dir ', round(nor,2),' ('+str(round(dx,2))+','+str(round(dx,2))+') is ', round(dori,2))
                    nearbyNodes.append(n)
    #nodetime=time.time()-st1
    #st1=time.time()
    nearbyWalls=[]
    for n in nearbyNodes:
        i=nearbyNodes.index(n)
        for n1 in n.cn:
            if(nearbyNodes.__contains__(n1)):
                i1=nearbyNodes.index(n1)
                if(i1>i):
                    nearbyWalls.append(NodalFunc(n,n1))
            else:
                nearbyWalls.append(NodalFunc(n,n1))

    #walltime=time.time()-st1

    #print 'nearby nodes : '+str(len(nearbyNodes))+"/"+str(len(nodes))+" || nearby walls : "+str(len(nearbyWalls))
    #print 'percent nodes : '+str(100.0*(nodetime/(nodetime+walltime)))

    maxorientdif=3.14159/18
    directions=[]
    standardWallLength=1.5*lscale/scale
    for w in nearbyWalls:
        dir=w.orient
        isNew=True
        mostsimilarindex=-1
        highestsim=1000
        lengthMult = w.getLength()/standardWallLength
        for d in directions:
            dir1=d[0]
            dirdif=abs(dir1-dir)
            dirdif1=abs(dir1-(dir-3.14159))
            dirdif2=abs(dir1-(dir+3.14159))
            if(dirdif1<dirdif):
                dirdif=dirdif1
            if(dirdif2<dirdif):
                dirdif=dirdif2
            if(dirdif<maxorientdif):
                if(dirdif<highestsim):
                    mostsimilarindex=directions.index(d)
                    highestsim=dirdif

        if(mostsimilarindex==-1):
            directions.append([dir, 1])
        else:
            d1 = dir
            if (abs(dir + 3.14 - dir1) < abs(d1 - dir1)): d1 = dir + 3.14159
            if (abs(dir - 3.14 - dir1) < abs(d1 - dir1)): d1 = dir - 3.14159
            directions[mostsimilarindex][0] = directions[mostsimilarindex][1] * directions[mostsimilarindex][0] + (d1*lengthMult)
            directions[mostsimilarindex][1] = directions[mostsimilarindex][1] + lengthMult
            directions[mostsimilarindex][0] = directions[mostsimilarindex][0] / directions[mostsimilarindex][1]

    routedirs = []
    for d in directions:
        if (d[1]>3):
            routedirs.append(directionalData(d))

    checkDirIntersects(car.x, car.y)
    dt=time.time()-st
    subtimes[4]=dt
    #print 'directions : ',directions


class directionalData():
    dir=0
    instances=0
    intercepts=[]

    def __init__(self, direction = [0,0]):
        self.dir=direction[0]
        self.intercepts=[]
        self.instances=direction[1]

    def toString(self):
        return 'dir ' + str(round(self.dir,4)) + ' (o=' + str(round((self.dir*180.0/3.14159),2)) + ') \n has ' + str(len(self.intercepts)) + ' intersections and '+str(self.instances)+" instances "


def checkDirIntersects(carx,cary):
    global dirIntersects, routedirs
    dirIntersects=[]
    for d in routedirs:
        intersects=intersectsInDirFromCar(d.dir,carx,cary)
        for i in intersects:
            dirIntersects.append(i)
            d.intercepts.append(i)



def intersectsInDirFromCar(dir,crx,cry):
    global nearbyWalls, nearbyNodes
    mx=np.math.tan(dir)
    b=cry-(mx*crx)
    intersects=[]
    for w in nearbyWalls:
        p=getLineWallIntersection(mx,b,w)
        if(p[0]>w.getMinMax(True, False) and p[0]<w.getMinMax(True, True)):
            if (p[1] > w.getMinMax(False, False) and p[1] < w.getMinMax(False, True)):
                intersects.append(Point(p[0],p[1]))
    return intersects

def getLineWallIntersection(m,b,wall):
    wm1 = (wall.n1.p.y - wall.n2.p.y) / ((wall.n1.p.x - wall.n2.p.x) + .000001)
    wb1 = wall.n1.p.y - (wm1 * wall.n1.p.x)
    x = (b - wb1) / (wm1 - m)
    y = (x * wm1) + wb1
    #solve using inverse: x=my+b
    #wm2 = (wall.n1.p.x - wall.n2.p.x) / ((wall.n1.p.y - wall.n2.p.y) + .000001)


    return [x,y]

def getSubTimes(subtimes):
    t=0
    for i in subtimes:
        t=t+i
    percent=' [ '
    for i in subtimes:
        p=i/t
        p=round(p*100,1)
        percent=percent+str(p)+'% '
    percent=percent+']'
    return str(round(t,4))+str(percent)


def getClosestCarPath(n1,n2):
    global carpath
    closest=[]
    closestd=100000
    closestnode=None
    if(carpath==None):return
    if(len(carpath.path)<3):return
    end=len(carpath.path)-1
    for p in itertools.islice(carpath.path,0,end):
        avgd=(n1.distToNode(p)+n2.distToNode(p))/2.0
        if (avgd<closestd):
            closestnode=p
            closestd=avgd
    if (closestnode!=None):
        closest.append(closestnode)
        index=carpath.path.index(closestnode)
        adj1=carpath.path[index+1]
        adj2=carpath.path[index-1]
        closestnode2=adj1
        if(closestnode.distToNode(adj1)>closestnode.distToNode(adj2)):closestnode2=adj2
        closest.append(closestnode2)
    return closest

def findPPaths():
    global ppaths, nodes, subtimes, methodIterations
    if(nodes<9):return
    #ppaths=[]
    #TODO IMPLEMENT NEW OLDNODE SYSTEM
    st=time.time()
    range=30
    end=len(nodes)-10
    if (end<0):end=len(nodes)
    start=end-range
    if(start<0):start=0
    for p in ppaths:
        if (not nodes.__contains__(p[0])):
            ppaths.remove(p)
        elif (not nodes.__contains__(p[1])):
            ppaths.remove(p)
    #start=1
    #end=int(len(nodes)*.9)
    for n in itertools.islice(nodes,start,end):
        index=nodes.index(n)
        for n1 in n.cn:
            if(nodes.__contains__(n1)):
                index1=nodes.index(n1)
                if (index1>index):
                    cpath=getClosestCarPath(n,n1)
                    if(cpath!=None):
                        if len(cpath)==2:
                            wallTangent=NodalFunc(n,n1)
                            pathTangent=NodalFunc(cpath[0],cpath[1])
                            deltaOrient=wallTangent.getOrientDif(pathTangent)
                            if(abs(deltaOrient)>45):
                                ppaths.append([n,n1])
    subtimes[5]=round((time.time()-st),3)
    methodIterations[2]=methodIterations[2]+1



def updatePath(pos,dl,dr,df):
    global carpath,timeOfLastLoop
    distdata=[dl,dr,df]
    if carpath==None:
        timeOfLastLoop=time.time()
        carpath=Path(pos,distdata)
    else:
        carpath.addPos(pos,distdata)

def combineDuplicates(n1, range):
    global nodes
    if (len(n1.cn) > 1):
        end = len(nodes)
        start = end - range
        if start < 0: start = 0
        for n in itertools.islice(nodes, start, end):
            if (n != n1):
                if (n1.equals(n)):
                    n1.combineNodes(n)
                    nodes.remove(n)

def distToClosestNode(n1, range):
    global nodes
    end = len(nodes)
    start = end - range
    if start < 0: start = 0
    x=n1.p.x
    y=n1.p.y
    closest = None
    closest_dist = 10000
    for n in itertools.islice(nodes,start, end):
        dx = n.p.x - x
        dy = n.p.y - y
        dist = np.math.sqrt(dx * dx + dy * dy)
        if (dist < closest_dist):
            closest_dist = dist
    return closest_dist

def getClosestNode(n1, range):
    global nodes
    end = len(nodes)
    start = end - range
    if start < 0: start = 0
    x=n1.p.x
    y=n1.p.y
    closest = None
    closest_dist = 10000
    for n in itertools.islice(nodes,start, end):
        dx = n.p.x - x
        dy = n.p.y - y
        dist = np.math.sqrt(dx * dx + dy * dy)
        if (dist < closest_dist):
            closest=n
            closest_dist = dist
    return closest

def removeImpossibleConnections(rangepercent):
    global nodes, oldlocs

def setConnections():
    global connections, nodes
    connections=[]
    for n in nodes:
        index=nodes.index(n)
        for n1 in n.cn:
            index1=nodes.index(n1)
            if (index1>index):
                connections.append(NodalConnection(n,n1))


def isDuplicateNode(n1,range):
    global nodes
    if (len(n1.cn)>1):
        end = len(nodes)
        start = end - range
        if start < 0: start = 0
        for n in itertools.islice(nodes, start, end):
            if (n!=n1):
                if (n1.equals(n)):
                    return True
    return False

def combineSimilarLines():
    print ("there are currently ", len(wall)," walls")
    s=time.time()
    maxdist=5.0*lscale/scale


    for l in wall:
        if (False == wall.__contains__(l)):continue
        #if (len(wall)>20 and wall.index(l)>len(wall)/2):continue
        for l1 in wall:
            if (wall.__contains__(l1) and wall.__contains__(l)):
                if (wall.index(l1)>wall.index(l)):
                    if (l1!=l):
                        sim=getLineOrientSimilarity(l,l1)
                        if (sim>.7 and sim<1.01):
                            dist=getDistBetweenLines(l,l1)
                            if (maxdist>dist):
                                print ("sim is ", sim, " and dist is ", dist, " so combining "''',l, ", ",l1''')
                                wall.append(combineLines(l, l1))
                                wall.remove(l)
                                wall.remove(l1)
                #break
    t=time.time()-s
    print ("combining took ",t," and there are now ",len(wall)," walls")


def clipLinesInterSecting(l1,right):
    for w in allwalls:
        if (doLinesIntersect(l1, w)):
            clipAtIntersect(w,l1,right)

def getDistBetweenLines(l1,l2):
    xs = [l1.p1.getX(), l1.p2.getX(), l2.p1.getX(), l2.p2.getX()]
    ys = [l1.p1.getX(), l1.p2.getX(), l2.p1.getX(), l2.p2.getX()]
    xs.remove(max(xs))
    xs.remove(min(xs))
    ys.remove(max(ys))
    ys.remove(min(ys))
    dx=xs[0]-xs[1]
    dy=ys[0]-ys[1]
    return np.math.sqrt(dx*dx+dy*dy)

def combineLines(l1,l2):
    xs=[l1.p1.getX(),l1.p2.getX(),l2.p1.getX(),l2.p2.getX()]
    ys=[l1.p1.getY(),l1.p2.getY(),l2.p1.getY(),l2.p2.getY()]
    if((l1.p1.getX()<l1.p2.getX()) == (l1.p1.getY()<l1.p2.getY())):
        return Line(Point(min(xs),min(ys)),Point(max(xs),max(ys)))
    else:
        return Line(Point(min(xs),max(ys)),Point(max(xs),min(ys)))


def getLineSimilarity(l1,l2):
    m1=(l1.p2.getX()-l1.p1.getX())/(l1.p2.getY()-l1.p1.getY())
    m2=(l2.p2.getX()-l2.p1.getX())/(l2.p2.getY()-l2.p1.getY())
    if (m1>0!=m2>0):
        return 0
    if (m1>m2):
        return (m2/m1)
    else:
        return (m1/m2)

def getSlopeSimilarity(m1,m2):
    if (m1>0!=m2>0):
        return 0
    if (m1>m2):
        return (m2/m1)
    else:
        return (m1/m2)

def getLineOrientSimilarity(l1,l2):
    m1=np.math.atan((l1.p2.getY()-l1.p1.getY())/(l1.p2.getX()-l1.p1.getX()))
    m2=np.math.atan((l2.p2.getY()-l2.p1.getY())/(l2.p2.getX()-l2.p1.getX()))

    if (m1>0!=m2>0):
        return 0
    if (m1>m2):
        return (m2/m1)
    else:
        return (m1/m2)



def getLineOrient(l):
    dx=l.p2.getX()-l.p1.getX()
    dy=l.p2.getY()-l.p1.getY()
    o=np.math.atan2(dy,dx)
    return o

def getClosestPoint(p1, exclude=None):
    global points
    x=p1.getX()
    y=p1.getY()
    closest=None
    closest_dist=10000
    for p in points:
        if (p!=p1 and p!=exclude):
            dx=p.getX()-x
            dy=p.getY()-y
            dist=np.math.sqrt(dx*dx+dy*dy)
            if(dist<closest_dist):
                closest=p
                #print ("new closest dist is ",closest_dist)
                closest_dist=dist

    #global scale, lscale
    #if (dist>6*lscale/scale):
    #    return None

    return closest

def getClosestPointInList(p1, list, exclude=None):
    global points
    x=p1.getX()
    y=p1.getY()
    closest=None
    closest_dist=10000
    for p in list:
        if (p!=p1 and p!=exclude):
            dx=p.getX()-x
            dy=p.getY()-y
            dist=np.math.sqrt(dx*dx+dy*dy)
            if(dist<closest_dist):
                closest=p
                #print ("new closest dist is ",closest_dist)
                closest_dist=dist

    #global scale, lscale
    #if (dist>6*lscale/scale):
    #    return None

    return closest


def doLinesIntersect(l1,l2):

    m1 = (l1.p2.getX() - l1.p1.getX()) / ((l1.p2.getY() - l1.p1.getY())+.000001)
    m2 = (l2.p2.getX() - l2.p1.getX()) / ((l2.p2.getY() - l2.p1.getY())+.000001)
    b1=l1.p1.getY()-(m1*l1.p1.getX())
    b2=l2.p1.getY()-(m2*l2.p1.getX())
    x=(b2-b1)/(m1-m2)

    if (l1.p1.x<x<l1.p2.x or l1.p1.x>x>l1.p2.x):
        if (l2.p1.x < x < l2.p2.x or l2.p1.x > x > l2.p2.x):
            #print (l1,", ",l2," returning true")
            return True
    #print("returning false")
    return False


def clipAtIntersect(l1,l2,right):
    intp=getIntersectPoint(l1,l2)
    rp=l1.p1
    if (rp.getX()<l1.p2.getX() and right):
        rp=l1.p2
    allwalls.remove(l1)
    allwalls.append(Line(rp,intp))

def getIntersectPoint(l1,l2):
    m1 = (l1.p2.getX() - l1.p1.getX()) / ((l1.p2.getY() - l1.p1.getY()) + .000001)
    m2 = (l2.p2.getX() - l2.p1.getX()) / ((l2.p2.getY() - l2.p1.getY()) + .000001)
    b1 = l1.p1.getY() - (m1 * l1.p1.getX())
    b2 = l2.p1.getY() - (m2 * l2.p1.getX())
    x = (b2 - b1) / (m1 - m2)
    y = (x*m1)+b1
    return Point(x,y)


def avgPoints():
    i=0
    global points
    for p in range(len(points)):
        if (i>1 and i+1<len(points)):
            x=(points[p-1].getX()+points[p].getX()+points[p+1].getX())/3
            y=(points[p-1].getY()+points[p].getY()+points[p+1].getY())/3
            points[p]= Point(x,y)
        i=i+1


def cleanPoints():
    global points,lscale, scale
    md=.2*lscale/scale #md = max distance
    mindist=.8*lscale/scale
    for p in points:
        if (not points.__contains__(p)):continue
        x=p.getX()
        y=p.getY()
        #print (x,y)
        for p1 in points:
            if (p1!=p and points.__contains__(p1) and points.__contains__(p)):
                x1 = p1.getX()-x
                y1 = p1.getY()-y
                dist=np.sqrt(x1*x1+y1*y1)


                if(dist<md):
                    newp=Point((p.x+p1.x)/2,(p.y+p1.y)/2)
                    points.remove(p1)
                    points.remove(p)
                    points.append(newp)

        '''close=getClosestPoint(p)
        if(close!=None):
            dist=distBetween(p,close)
            if (dist>mindist):
                points.remove(p)
        '''


def distBetween(p1=Point(0,0),p2=Point(0,0)):
    dx=p1.getX()-p2.getX()
    dy=p1.getY()-p2.getY()
    return np.sqrt(dx*dx+dy*dy)


def getPoint(data, dir):
    global scale, orient,lscale
    r=get_dist(data,dir)
    #d=get_data_array(data, dir-3,dir+3)
    #dir=240-dir
    if (r<8):
        global x,y,points
        r=r*lscale/scale
        d1=((((dir)-120)*3.14)/180.0)+orient
        #r=d.mean()*lscale/scale
        p=Point(x / scale + (r * np.math.cos(d1)), y / scale + (r * np.math.sin(d1)))
        return p
    return None

def get_dist(data, a=0):
    PPD = 4.5  # points per degree
    return data[int(a * PPD)]

def get_avg_dist(data, a=0):
    PPD = 4.5  # points per degree
    return get_data_array(data,a-10,a+10).mean()
    #return data[int(a * PPD)]

def get_data_array(data, a=0, b=240):
    PPD = 4.5  # points per degree
    data = data[int(a * PPD):int(b * PPD)]
    arr = np.array(data)
    return arr

def move(amt):
    global x,y, orient
    x=x+ (amt * np.math.cos(orient))
    y=y + (amt * np.math.sin(orient))

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def getSlopeOf(l1):
    return (l1.p2.getY() - l1.p1.getY()) / ((l1.p2.getX() - l1.p1.getX()))

def connectNodes(list):
    global scale,lscale,nodes,subtimes, methodIterations
    st=time.time()
    maxdist=1.2*lscale/scale
    for n in list:
        #n.printNode()
        if (n.hasDisconnect()):
            n.connectWithClosest(list,maxdist,0)
            #if(n.hasNoConnected()):
            #    list.remove(n)
    subtimes[2]=round((time.time()-st),3)
    return list

def connectVeryClose():
    global nodes, scale, lscale
    rad=.5*lscale/scale
    for n in nodes:
        for n1 in nodes:
            if (n1!=n):
                if (not n1.contains(n)):
                    dist=n.distToNode(n1)
                    if(dist<rad):
                        connectTwoNodes(n,n1)


def connectTwoNodes(n1,n2):
    n1.tryAddNode(n2)
    n2.tryAddNode(n1)

def straighten(range):
    pass


def simplifyPath():
    global carpath
    #TODO: Use this to test simplify

def simplify(range=20):
    global nodes,scale,lscale
    maxdist=5*lscale/scale
    end=len(nodes)
    start=end-range
    if start<0:start=0
    #print("range = ",start,'-',end)
    for n in itertools.islice(nodes,start,end):

        if(not n.hasDisconnect()):
            if(len(n.cn)==2):
                dist=n.cn[0].distToNode(n.cn[1])

                if(dist<maxdist):
                    #distToFirst = (n.distToNode(n.cn[1]) + n.distToNode(n.cn[0])) / 2.0
                    #if dist > distToFirst:
                    if True:
                        '''nfunc1=NodalFunc(n,n.cn[0])
                        nfunc2=NodalFunc(n,n.cn[1])
                        odif=nfunc1.getOrientDif(nfunc2)
                        if odif<45:'''
                        #TODO MAKE  SURE THIS WORKS
                        #adif1=getAngDif(n.getAngToNode(n.cn[0]),n.getAngToNode(n.cn[1]))
                        #if(adif1<3.14/4):
                        if(True):
                            #print ("removing ", n.printNode())
                            n.cn[0].replaceNWith(n, n.cn[1])
                            n.cn[1].replaceNWith(n, n.cn[0])
                            if (nodes.__contains__(n)):
                                nodes.remove(n)
    pass

def cleanNodes(range=40):
    global subtimes,methodIterations
    st=time.time()

    removeTriangles(range)
    #removeBranches(range)
    removeTwinNodes(range)
    #resetLargeNodes(range)
    #removeAllDupes()
    simplify(range)
    archiveOldNodes()
    subtimes[3]=round((time.time()-st),3)

def removeAllDupes():
    global nodes
    for n in nodes:
        n.removeDupes()

def archiveOldNodes():
    global nodes, oldNodes, methodIterations,subtimes
    st=time.time()
    archivetime=25
    for n in nodes:
        if(methodIterations[0]-n.iterationOfCreation>archivetime):
            newest=n.getNewestInNetwork()
            if(methodIterations[0]-newest.iterationOfCreation>archivetime):
                #oldNodes.append(n)
                n.archive()
                #nodes.remove(n)
    subtimes[6]=time.time()-st

def getTotalNodeData():
    global nodes,oldNodes
    leng=len(nodes)
    avgsize=getAvgNodeNet()
    nodelen=[0,0,0,0,0,0,0]
    for n in nodes:
        nl=len(n.cn)
        if nl<7:
            nodelen[nl]=nodelen[nl]+1
    for n in oldNodes:
        nl=len(n.cn)
        if nl<7:
            nodelen[nl]=nodelen[nl]+1
    str1=str(leng)+'-'+str(len(oldNodes))+' nodes, size distribution : 0-'+str(nodelen[0])+', 1-'+str(nodelen[1])+', 2-'+str(nodelen[2])+', 3-'+str(nodelen[3])+', 4-'+str(nodelen[4])+', 5-'+str(nodelen[5])+', 6-'+str(nodelen[6])
    return str1


def getAvgNodeNet():
    global nodes
    if len(nodes)==0:return
    sum=0
    for n in nodes:
        sum=sum+len(n.cn)

    return (sum/(len(nodes)))

def removeTwinNodes(range=20):
    global nodes,scale,lscale
    mdist=.1*lscale/scale
    #for line in itertools.islice(list, start, stop):
    #    foo(line)
    end=len(nodes)
    start=end-range
    if start<0:start=0
    for n in itertools.islice(nodes,start,end):
        n.removeNodesTooClose(mdist)


def removeAbsentNodes():
    global nodes, oldNodes
    for n in nodes:
        #n.removeDuplicateCN
        for n1 in n.cn:
            if not nodes.__contains__(n1) and not oldNodes.__contains__(n):

                #print ("phantom node deleted")
                n.cn.remove(n1)

def removeTriangles(range=20):
    global nodes
    end=len(nodes)
    start=end-range
    if start<0:start=0
    for n in itertools.islice(nodes,start,end):
        if len(n.cn)==2:
            if n.cn[0].contains(n.cn[1]) and n.cn[1].contains(n.cn[0]):
                n.cn[0].removeNode(n)
                if(len(n.cn)==2):
                    n.cn[1].removeNode(n)
                else:
                    n.cn[0].removeNode(n)
                nodes.remove(n)

def removeBranches(range=20):
    global nodes
    end=len(nodes)
    start=end-range
    if start<0:start=0
    for n in itertools.islice(nodes,start,end):

        if len(n.cn) == 1:
            if (len(n.cn[0].cn)>2):
                n.cn[0].removeNode(n)
                nodes.remove(n)

def resetLargeNodes(range=20):
    global nodes, scale, lscale
    maxnewdist = 1.5 * lscale / scale
    minkeepdist = 2 * lscale / scale
    global nodes
    #end=len(nodes)
    #start=end-range
    for n in nodes:
        if len(n.cn)>3:
            n.resetNode(nodes,maxnewdist,minkeepdist)

def correctPositionalDrift(data):
    global similarpos,x,y,scale,lscale, lastCorrection
    maxDist=100000
    if(len(similarpos)==0):return
    possNewPos=[]
    for p in similarpos:
        datasim = getDataSim(data, p.data)
        print 'data sim = ', datasim, '%'
        if(datasim>.8):
            possNewPos.append([p.p.x*scale,p.p.y*scale])
    if(len(possNewPos)==0):
        return
    closest=[100000,100000]
    closestDist=1000000
    for p in possNewPos:
        dx=p[0]-x
        dy=p[1]-y
        dist=np.math.sqrt(dx*dx+dy*dy)
        if(dist<closestDist):
            closest=p
            closestDist=dist

    #print 'closest pos = ',closestDist
    if(closestDist<120 and closestDist>30):
        print ('moved from ',x,',',y,' to ',closest)
        #haslapped=1
        dx=(closest[0]-x)/scale
        dy=(closest[1]-y)/scale
        correctRecentPoints(dx,dy)
        lastCorrection = time.time()
        x=(closest[0])
        y=(closest[1])

def correctRecentPoints(dx,dy,range=20):
    global carpath
    end=len(carpath.path)
    start=end-range
    if(start<0):start=0
    rr=end-start
    percent=0
    for n in itertools.islice(carpath.path,start,end):
        percent=percent+(1.0/rr)
        dxn=dx*percent
        dyn=dy*percent
        n.p=Point(n.p.x+dxn,n.p.y+dyn)

def getDataSim(d1,d2):
    p=0
    for i in range(3):
        phere=(d1[i]/d2[i])
        if(d1[i]>d2[i]):
            phere=(d2[i]/d1[i])
        p=p+phere
    p=p/3.0
    return p

def getSimilarPos(dl,dr,df):
    global carpath,similarpos
    similarpos=[]
    data=[dl,dr,df]
    if (carpath!=None):
        if(len(carpath.path)>10):
            last=carpath.path[len(carpath.path)-1]
            for n in carpath.path:
                if (n!=last):
                    d1=n.dir/3.14*180
                    d2=last.dir/3.14*180
                    orientDif=abs(d1-d2)
                    o1=abs((d2)-(d1+360))
                    o2=abs((d2)-(d1-360))
                    if(o1<orientDif):orientDif=o1
                    if(o2<orientDif):orientDif=o2
                    if(orientDif<20):
                        d1 =getDirFromN1toN2(last,n)
                        orientDif = abs(d1 - d2)
                        o1 = abs((d2) - (d1 + 360))
                        o2 = abs((d2) - (d1 - 360))
                        if (o1 < orientDif): orientDif = o1
                        if (o2 < orientDif): orientDif = o2
                        if(abs(orientDif)>60 and abs(orientDif)<120):
                            similarpos.append(n)
            correctPositionalDrift(data)


def getAngDif(a1,a2):
    adif = abs(a1 - a2)
    if (adif > 3.14159):
        adif = abs(a1 + (3.14159 * 2) - a2)
    if (adif > 3.14159):
        adif = a1 - (3.14159 * 2) - a2
    return adif

def getDirFromN1toN2(n1,n2):
    dx=n2.p.getX()-n1.p.getX()
    dy=n2.p.getY()-n1.p.getY()
    orient=np.math.atan(dy/dx)*180/3.14
    if(dx<0):orient=orient+180
    return orient

class carState():
    x=0
    y=0
    orient=0
    def __init__(self, x,y,orient):
        self.x=x
        self.y=y
        self.orient=orient
    def update(self, x,y,orient):
        self.x=x
        self.y=y
        self.orient=orient

class Path():
    path=[]

    def __init__(self,pos,data):
        global orient
        n=Node(pos)
        n.dir=orient
        n.data=data
        self.path=[n]

    def addPos(self, pos, data):
        global orient
        n=Node(pos)
        n.data=data
        n.dir=orient
        lastNode=None
        pathlen=len(self.path)
        if (pathlen>0):
            lastNode=self.path[pathlen-1]
        n.tryAddNode(lastNode)
        lastNode.tryAddNode(n)
        self.path.append(n)

    def getOrientLines(self):
        lines=[]
        for n in self.path:
            p1=n.p
            x1=n.p.getX()+(20*np.math.cos(n.dir))
            y1=n.p.getY()+(20*np.math.sin(n.dir))
            l=Line(p1,Point(x1,y1))
            l.setArrow("last")
            lines.append(l)
        return lines

    def getLines(self):
        lines=[]
        for n in self.path:
            nl=n.getLines()
            for l in nl:
                lines.append(l)
        return lines


class Node():
    p=None
    data=None
    dir=0
    cn=[]
    iterationOfCreation=0

    def __init__(self,p1):
        global methodIterations
        #global scale, lscale,nodes
        #maxInit=.5*lscale/scale
        self.p=p1
        self.cn=[]
        self.iterationOfCreation=methodIterations[0]
        #self.connectWithClosest(nodes,maxInit)


    def areConnectedInList(self, list):
        for n in self.cn:
            if(not list.__contains__(n)):
                return False
        return True

    def hasNoConnected(self):
        return len(self.cn)==0

    def removeSelfFromCN(self):
        if(self.cn.__contains__(self)):
            self.cn.remove(self)

    def removeNodesTooClose(self,mdist):
        global nodes
        for n in self.cn:
            dist=self.distToNode(n)
            if(dist<mdist):
                self.cn.remove(n)
                n.removeNode(self)
                if (nodes.__contains__(n)):
                    nodes.remove(n)
                for node in n.cn:
                    if node!=self:
                        if not self.cn.__contains__(node):
                            self.cn.append(node)

#    def getNewestConnected(self):
#        for n in n.cn
    def getNetwork(self, clist=[]):
        if(not clist.__contains__(self)):
            clist.append(self)
            for n in self.cn:
                clist=n.getNetwork(clist)
        return clist

    def getNewestInNetwork(self):
        net=self.getNetwork()
        newest=None
        for n in net:
            if(newest==None):
                newest=n
            else:
                if(newest.iterationOfCreation<n.iterationOfCreation):
                    newest=n
        return n

    def archive(self):
        global nodes, oldNodes
        if(nodes.__contains__(self)):
            nodes.remove(self)
            oldNodes.append(self)
            #for n in self.cn:
            #    n.archive()

    def removeDuplicateCN(self):
        ncn=[]
        for n in self.cn:
            if (not ncn.__contains__(n)):
                ncn.append(n)

    def replaceNWith(self,cn,nn):
        if self.cn.__contains__(cn):
            self.cn.remove(cn)
            self.cn.append(nn)

    #def getClosestFour(self, nodes):
        #cln=[None, None, None]

    def removeOtherConnected(self):
        global nodes
        for n in self.otherConnected:
            if nodes.__contains__(n):
                if(n.isNodeTheOnlyConnected(self)):
                    nodes.remove(n)
                else:
                    n.removeNode(self)

            else:
                self.otherConnected.remove(n)

    def printNode(self):
        print 'node (',self.p,') is attatched to '
        for n in self.cn:
            print(n.p)

    def removeDupes(self):
        dupes1=[]
        for n in self.cn:
            numsim=0
            for n1 in self.cn:
                if (self.cn.index(n1)>self.cn.index(n)):
                    if (n1.equals(n)):
                        numsim=numsim+1
                        if (numsim>1):
                            dupes1.append(n1)
        for n in dupes1:
            self.cn.remove(n)

    def removeNode(self,node):
        if(self.cn.__contains__(node)):
            self.cn.remove(node)

    def resetNode(self,list,maxdist,maxkeepdist):
        for node in self.cn:
            if node.getSize()>2:
                if self.distToNode(node)<maxkeepdist:
                    node.removeNode(self)
                    self.cn.remove(node)
        self.connectWithClosest(list,maxdist)

    def isNodeTheOnlyConnected(self,node):
        return len(self.cn)==1 and self.cn.__contains__(node)
        #return (self.n1==node and self.n2==None) or (self.n2==node and self.n1==None)

    def doConnectedNodesConnectBack(self):
        #for n in self.cn:
        pass
        #return self.n1.contains(self) and self.n2.contains(self)

    def hasDisconnect(self):
        if len(self.cn)<2:
            return True
        return False
        #return self.n1==None or self.n2==None

    def getSize(self):
        return len(self.cn)

    def connectWithClosest(self,nodes, maxdist, indexLimited=0):
        start=0
        end=len(nodes)
        if(indexLimited==1):
            if(nodes.__contains__(self)):
                start=nodes.index(self)
        if (len(self.cn)==0):
            c1=None
            c1dist = 100000
            for n in itertools.islice(nodes,start,end):
                if n != self:
                    dx = (self.p.x - n.p.x)
                    dy = (self.p.y - n.p.y)
                    if(dx<maxdist and dy<maxdist):
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < c1dist:
                            #print("replace ",c1dist,' with ',dist)
                            c1 = n
                            c1dist = dist
            if c1dist < maxdist and c1 != None:
                #if (c1.tryAddNode(self)):
                c1.tryAddNode(self)
                self.tryAddNode(c1)
                #self.cn.append(c1)
        if (len(self.cn)==1):
            c2 = None
            c2dist = 100000
            for n in itertools.islice(nodes,start,end):
                if n != self and (not self.cn.__contains__(n)):
                    dx = (self.p.x - n.p.x)
                    dy = (self.p.y - n.p.y)
                    if (dx < maxdist and dy < maxdist):
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < c2dist:
                            c2 = n
                            c2dist = dist
            if c2dist < maxdist and c2 != None:
                c2.tryAddNode(self)
                self.tryAddNode(c2)
               # self.cn.append(c2)

    def equals(self,node1):
        if (len(self.cn)==len(node1.cn)):
            for n in self.cn:
                if(not node1.cn.__contains__(n)):
                    return False
            return True
        return False

    def combineNodes(self,n1):
        x=(self.p.x+n1.p.x)/2.0
        y=(self.p.y+n1.p.y)/2.0
        self.p=Point(x,y)
        for n in n1.cn:
            if (not self.cn.__contains__(n)):
                self.cn.append(n)

    def tryAddNode(self, n):
        if (not self.cn.__contains__(n)):
            self.cn.append(n)

    def getAngToNode(self, n):
        dx=n.p.x-self.p.x
        dy=n.p.y-self.p.y
        a=np.arctan2(dy,dx)
        return a

    def distToNode(self, n):
        dx = self.p.x - n.p.x
        dy = self.p.y - n.p.y
        return np.sqrt(dx*dx+dy*dy)

    def contains(self, node):
        return self.cn.__contains__(node)

    def getLines(self):
        lines=[]
        for n in self.cn:
            lines.append(Line(self.p,n.p))
        return lines

class NodalConnection():
    n1=None
    n2=None
    def __init__(self,n1,n2):
        self.n1=n1
        self.n2=n2
    def doesStillExist(self):
        global nodes
        if (nodes.__contains__(self.n1) and nodes.__contains__(self.n2)):
            if (self.doNodesConnect()):
                return True
        return False
    def doNodesConnect(self):
        return (self.n1.contains(self.n2) and self.n2.contains(self.n1))

class NodalFunc():
    m = None  # slope
    b = None  # y intercept
    orient = None
    n1=None
    n2=None

    def __init__(self, n1,n2):
        self.n1=n1
        self.n2=n2
        if(n1.p.x==n2.p.x):
            self.m=9999
        else:
            self.m = (n2.p.y - n1.p.y) / ((n2.p.x - n1.p.x))
        self.b = n1.p.y - (n1.p.x * self.m)
        self.orient = np.math.atan(self.m)

    def f(self, x):
        return (self.m * x) + self.b

    def getLength(self):
        dx=self.n1.p.x-self.n2.p.x
        dy=self.n1.p.y-self.n2.p.y
        return np.math.sqrt(dx*dx+dy*dy)

    def getOrientDif(self, nfunc):
        o1 = np.rad2deg(self.orient) + 90
        o2 = np.rad2deg(nfunc.orient) + 90
        if (o1 > o2):
            o1 = np.rad2deg(nfunc.orient) + 90
            o2 = np.rad2deg(self.orient) + 90
        dor = abs(o1 - o2)
        dor1 = abs((o1 + 180) - o2)
        if (dor < dor1):
            return dor
        return dor1

    def getCenter(self):
        return Point((self.n1.p.x + self.n2.p.x) / 2.0, (self.n1.p.y + self.n2.p.y) / 2.0)

    def getDistToMidPoint(self, line):
        return self.getDistToPoint(line.getCenter())

    def getDistToPoint(self, p=Point(0, 0)):
        x = p.getX()
        y = p.getY()
        m1 = -1.0 / self.m
        b1 = y - (m1 * x)
        x1 = (self.b - b1) / (m1 - self.m)
        y1 = self.f(x1)
        dx = x - x1
        dy = y - y1
        return np.sqrt(dx * dx + dy * dy)

    def getMinMax(self,isX,isMax):
        if isX:
            if isMax:
                return self.n1.p.x if(self.n1.p.x>self.n2.p.x) else self.n2.p.x
            else:
                return self.n1.p.x if(self.n1.p.x<self.n2.p.x) else self.n2.p.x
        else:
            if isMax:
                return self.n1.p.y if(self.n1.p.y>self.n2.p.y) else self.n2.p.y
            else:
                return self.n1.p.y if(self.n1.p.y<self.n2.p.y) else self.n2.p.y


class PointLine():
    allpoints=[]
    p1=None
    p2=None

    def __init__(self,points, ep1,ep2):
        self.allpoints=points
        self.p1=ep1
        self.p2=ep2

    def sortLine(self):
        #print self.allpoints
        dx=self.p1.getX()-self.p2.getX()
        dy=self.p1.getY()-self.p2.getY()
        l=np.sqrt(dx*dx+dy*dy)

        sorted=[]
        num=len(self.allpoints)
        for i in range(num):
            sx=self.allpoints[0]
            for p in self.allpoints:
                if (self.locOnLine(p,l)<self.locOnLine(sx,l)):
                    sx=p
            sorted.append(sx)
            self.allpoints.remove(sx)
        self.allpoints=sorted

        #print sorted

    def locOnLine(self, p, l):
        fp1=(distBetween(p,self.p1)/l)
        fp2=1-(distBetween(p,self.p2)/l)
        return (fp1+fp2)/2.0

    def getSize(self):
        return len(self.allpoints)

class CombineLine():
    linefunc=None
    subLines=[]

    def __init__(self,l1):
        self.linefunc=LineFunc(l1)
        self.subLines=[]
        self.subLines.append(l1)

    def createSubLines(self,list):
        global scale,lscale
        for w in list:
            if (not self.subLines.__contains__(w)):
                self.tryAddLine(w)

    def removeSubLine(self, l1):
        if self.subLines.__contains__(l1):
            self.subLines.remove(l1)

    def tryAddLine(self, l1):
        global scale,lscale
        maxPerpDist=.2*lscale/scale
        maxODif=8
        odif=self.linefunc.getOrientDif(l1)
        if (odif>maxODif):
            return
        perpdist=self.linefunc.getDistToMidPoint(l1)
        if (perpdist<maxPerpDist):
            actualperpdist=perpdist*scale/lscale
            #print(l1," is a subline because perpdist = ",perpdist," (",actualperpdist,") and odif = ",odif)
            self.subLines.append(l1)

    def getLine(self):
        sp=self.subLines[0].p1
        ep=self.subLines[0].p1
        for l in self.subLines:
            if (l.p1.getX()<sp.getX()):
                sp=l.p1
            if (l.p2.getX()<sp.getX()):
                sp=l.p2
            if (l.p1.getX()>ep.getX()):
                ep=l.p1
            if (l.p2.getX()>ep.getX()):
                ep=l.p2
        return Line(sp,ep)

    def getSize(self):
        return len(self.subLines)

class LineFunc():

    m = None  # slope
    b = None  # y intercept
    orient=None

    def __init__(self,l1):
        #self.m=(l1.p2.getX() - l1.p1.getX()) / ((l1.p2.getY() - l1.p1.getY()))
        self.m=(l1.p2.getY() - l1.p1.getY()) / ((l1.p2.getX() - l1.p1.getX()))
        self.b=l1.p1.getY()-(l1.p1.getX()*self.m)
        self.orient=np.math.atan(1/self.m)

    def f(self,x):
        return (self.m*x)+self.b

    def getOrientDif(self, l1):
        func=LineFunc(l1)
        o1=np.rad2deg(self.orient)+90
        o2=np.rad2deg(func.orient)+90
        if (o1>o2):
            o1=np.rad2deg(func.orient)+90
            o2=np.rad2deg(self.orient)+90
        dor=abs(o1-o2)
        dor1=abs((o1+180)-o2)
        if (dor<dor1):
            return dor
        return dor1



    def isLineSim(self, l1, maxDistDif, maxOrientDif):
        func=LineFunc(l1)
        orientsim=1-(abs(self.orient-func.orient)/3.14)
        orientdif=abs(self.orient-func.orient)
        slopesim=getSlopeSimilarity(self.m,func.m)
        dy1=self.f(l1.p1.getX())-l1.p1.getY()
        dy2=self.f(l1.p2.getX())-l1.p2.getY()
        dy=abs((dy1+dy2)/2.0)
        if (orientdif<maxOrientDif and dy<maxDistDif):
            return True
        #if (slopesim>.85 and dy<maxDistDif):
        #    return True
        return False

    def getDistToMidPoint(self, line):
        return self.getDistToPoint(line.getCenter())

    def getDistToPoint(self, p=Point(0,0)):
        x=p.getX()
        y=p.getY()
        m1=-1.0/self.m
        b1=y-(m1*x)
        x1 = (self.b - b1) / (m1 - self.m)
        y1=self.f(x1)
        dx=x-x1
        dy=y-y1
        return np.sqrt(dx*dx+dy*dy)
    
    def __repr__(self):
        return "f(x)=".format(str(self.m),"x+",str(self.b))
def saveImg(imgnum):
    # saves the current TKinter object in postscript format
    win.postscript(file="track/image.eps", colormode='color')

    # Convert from eps format to gif format using PIL
    from PIL import Image as NewImage
    img = NewImage.open("track/image.eps")
    string='track/track'+str(imgnum)+'.gif'
    img.save(string, "gif")