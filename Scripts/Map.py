import numpy as np
import itertools
from graphics import *

win = GraphWin("My Circle", 1000, 1000)

c=None
front=None
scale=4
x=win.getWidth()/2*scale
y=win.getHeight()/2*scale
orient=0
lt=time.time()
lscale=50.0
scantime=0



oldLocs=[Point(x / scale, y / scale)]
points=[]
#points=[]
allwalls=[]
wall=[]
nodes=[]
newnodes=[]
carpath=None

ppaths=[]

def update(dir):
    global win,front,c,x,y,orient, oldLocs
    d1=dir/180.0*3.14
    orient=d1

def render():
    global win,front,c,x,y,orient, oldLocs,points,allwalls,scantime,wall,newnodes,carpath,ppaths
    c = Circle(Point(x/scale,y/scale), 7)
    c.setFill("black")
    front = Circle(Point(x / scale + (7 * np.math.cos(orient)), y / scale + (7 * np.math.sin(orient))), 5)
    front.setFill("red")

    if(carpath!=None):
        s1 = "recorded locations : ", len(carpath.path), " s "
        t1loc = Point(500, 50)
        t1 = Text(t1loc, s1)
    s2="last scan time : ",scantime," s "
    t2loc=Point(500,100)
    t2=Text(t2loc,s2)
    s3=getTotalNodeData()
    t3loc=Point(500,150)
    t3=Text(t3loc,s3)

    clear(win)
    if carpath!=None:
        path=carpath.getLines()
        for l in path:
            l.setFill("grey")
            l.draw(win)

    for p in ppaths:
        c1=Circle(p,5)
        c1.setFill("orange")
        c1.draw(win)
    for n in nodes:
        size=len(n.cn)
        rad=size/2.0+1
        p2 = Circle(n.p, rad)
        if size<6:
            cmult=size/5.0
            p2.setFill(color_rgb(250*cmult,0,250*(cmult)))#250*size,0,250*(1-size)
        else:
            p2.setFill("red")

        p2.draw(win)
        lines=n.getLines()
        for l in lines:
            #l.setFill("black")
            l.draw(win)

    c.draw(win)
    if (carpath!=None):
        t1.draw(win)
    t2.draw(win)
    t3.draw(win)
    front.draw(win)

def scanWalls(data):
    global orient,x,y,lt,oldLocs,points, scantime,allwalls,wall,nodes,newnodes, scale, lscale, nodes
    if (orient==0):return
    samples=20
    st=time.time()
    scanrange=50
    prange=len(nodes)*.8
    if prange>scanrange:scanrange=prange

    for i in range(samples):
        lp1=getPoint(data,i*(115/samples)+5)
        rp1=getPoint(data,240-(i*(115/samples)+5))
        if (lp1!=None):
            points.append(lp1)
        if (rp1!=None):
            points.append(rp1)

    mininitdist=.25*lscale/scale
    for p in points:
        n=Node(p)
        if (distToClosestNode(n,40)>mininitdist):
            nodes.append(n)
            newnodes.append(n)

    points=[]
    nodes = connectNodes(nodes)
    newnodes = []
    removeAbsentNodes()

    if (time.time() > lt + .5):
        cleanNodes(scanrange)
        lt = time.time()
        findPPaths()
        updatePath(Point(x / scale, y / scale))
        scantime = time.time() - st


def getClosestCarPath(n1,n2):
    global carpath
    closest=[]
    closestd=100000
    closestnode=None
    if(carpath==None):return
    for p in carpath.path:
        avgd=(n1.distToNode(p)+n2.distToNode(p))/2.0
        if (avgd<closestd):
            closestnode=p
            closestd=avgd
    if (closestnode!=None):
        closest.append(closestnode)
        closestnode2=None
        closestd=100000
        for p in carpath.path:
            if (p!=closestnode):
                avgd = (n1.distToNode(p) + n2.distToNode(p)) / 2.0
                if (avgd < closestd):
                    closestnode2 = p
                    closestd = avgd
        if (closestnode2!=None):
            closest.append(closestnode2)
    return closest

def findPPaths():
    global ppaths, nodes
    if(nodes<9):return
    ppaths=[]
    start=1
    end=int(len(nodes)*.9)
    for n in itertools.islice(nodes,start,end):
        index=nodes.index(n)
        for n1 in n.cn:
            index1=nodes.index(n1)
            if (index1>index):
                cpath=getClosestCarPath(n,n1)
                if(cpath!=None):
                    if len(cpath)==2:
                        wallTangent=NodalFunc(n,n1)
                        pathTangent=NodalFunc(cpath[0],cpath[1])
                        deltaOrient=wallTangent.getOrientDif(pathTangent)
                        if(abs(deltaOrient)>45):
                            ppaths.append(Point((n.p.x+n1.p.x)/2.0,(n.p.y+n1.p.y)/2.0))


def updatePath(pos):
    global carpath
    if carpath==None:
        carpath=Path(pos)
    else:
        carpath.addPos(pos)

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

def removeImpossibleConnections(rangepercent):
    global nodes, oldlocs


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
    global scale,lscale,nodes
    maxdist=1.2*lscale/scale
    for n in list:
        #n.printNode()
        if (n.hasDisconnect()):
            n.connectWithClosest(list,maxdist)
            #if(n.hasNoConnected()):
            #    list.remove(n)
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

def simplify(range=20):
    global nodes,scale,lscale
    maxdist=3*lscale/scale
    end=len(nodes)
    start=end-range
    if start<0:start=0
    #print("range = ",start,'-',end)
    for n in itertools.islice(nodes,start,end):

        if(not n.hasDisconnect()):
            if(len(n.cn)==2):
                dist=n.cn[0].distToNode(n.cn[1])

                if(dist<maxdist):
                    distToFirst = (n.distToNode(n.cn[1]) + n.distToNode(n.cn[0])) / 2.0
                    if dist > distToFirst:
                        '''nfunc1=NodalFunc(n,n.cn[0])
                        nfunc2=NodalFunc(n,n.cn[1])
                        odif=nfunc1.getOrientDif(nfunc2)
                        if odif<45:'''
                        #print ("removing ", n.printNode())
                        n.cn[0].replaceNWith(n, n.cn[1])
                        n.cn[1].replaceNWith(n, n.cn[0])
                        if (nodes.__contains__(n)):
                            nodes.remove(n)
    pass

def cleanNodes(range=40):
    removeTriangles(range)
    removeBranches(range)
    removeTwinNodes(range)
    resetLargeNodes(range)
    simplify(range)



def getTotalNodeData():
    global nodes
    leng=len(nodes)
    avgsize=getAvgNodeNet()
    nodelen=[0,0,0,0,0,0,0]
    for n in nodes:
        nl=len(n.cn)
        if nl<7:
            nodelen[nl]=nodelen[nl]+1
    str=leng,' nodes : 0-',nodelen[0],', 1-',nodelen[1],', 2-',nodelen[2],', 3-',nodelen[3],', 4-',nodelen[4],', 5-',nodelen[5],', 6-',nodelen[6]
    return str


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
    global nodes
    for n in nodes:
        #n.removeDuplicateCN
        for n1 in n.cn:
            if not nodes.__contains__(n1):

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

class Path():
    path=[]

    def __init__(self,pos):
        self.path=[Node(pos)]

    def addPos(self, pos):
        n=Node(pos)
        lastNode=None
        pathlen=len(self.path)
        if (pathlen>0):
            lastNode=self.path[pathlen-1]
        n.tryAddNode(lastNode)
        lastNode.tryAddNode(n)
        self.path.append(n)

    def getLines(self):
        lines=[]
        for n in self.path:
            nl=n.getLines()
            for l in nl:
                lines.append(l)
        return lines

class Node():
    p=None
    #n1=None
    #n2=None
    #otherConnected=[]
    cn=[]
    def __init__(self,p1):
        #global scale, lscale,nodes
        #maxInit=.5*lscale/scale
        self.p=p1
        self.cn=[]
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


    def removeDuplicateCN(self):
        ncn=[]
        for n in self.cn:
            if (not ncn.__contains__(n)):
                ncn.append(n)

    def replaceNWith(self,cn,nn):
        if self.cn.__contains__(cn):
            self.cn.remove(cn)
            self.cn.append(nn)

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

    def connectWithClosest(self,nodes, maxdist):
        if (len(self.cn)==0):
            c1=None
            c1dist = 100000
            for n in nodes:
                if n != self:
                    dx = (self.p.x - n.p.x)
                    dy = (self.p.y - n.p.y)
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
            for n in nodes:
                if n != self and (not self.cn.__contains__(n)):
                    dx = (self.p.x - n.p.x)
                    dy = (self.p.y - n.p.y)
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


class NodalFunc():
    m = None  # slope
    b = None  # y intercept
    orient = None
    n1=None
    n2=None

    def __init__(self, n1,n2):
        self.n1=n1
        self.n2=n2
        self.m = (n2.p.y - n1.p.y) / ((n2.p.x - n1.p.x))
        self.b = n1.p.y - (n1.p.x * self.m)
        self.orient = np.math.atan(1 / self.m)

    def f(self, x):
        return (self.m * x) + self.b

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

