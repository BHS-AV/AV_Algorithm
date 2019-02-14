import numpy as np

from graphics import *

#from Scripts.graphics import Circle, Point

win = GraphWin("My Circle", 1000, 1000)

#if __name__=="__main__":
    #global win
    #win = GraphWin("My Circle", 100, 100)
c=None
front=None
scale=4
x=win.getWidth()/2*scale
y=win.getHeight()/2*scale
orient=0
lt=time.time()
lscale=50.0


oldLocs=[Point(x / scale, y / scale)]
points=[]
#points=[]
wall=[]
allwalls=[]

def update(dir):
    global win,front,c,x,y,orient, oldLocs
    d1=dir/180.0*3.14
    orient=d1

def render():
    global win,front,c,x,y,orient, oldLocs
    c = Circle(Point(x/scale,y/scale), 10)
    c.setFill("black")
    s="walls : ",len(wall)," "
    tloc=Point(100,100)
    t=Text(tloc,s)
    front= Circle(Point(x/scale+(10*np.math.cos(orient)),y/scale+(10*np.math.sin(orient))),5)
    front.setFill("red")
    clear(win)
    for pp in oldLocs:
        p=Circle(pp, 3)
        p.setFill("orange")
        p.draw(win)

    for l in wall:
        l.draw(win)
    for w in allwalls:
        w.setFill("blue")

    c.draw(win)
    t.draw(win)
    front.draw(win)


def scanWalls(data):
    global orient,x,y,lt,oldLocs
    if (orient==0):return
    samples=20
    lp=None
    rp=None

    cp=Point(x/scale,y/scale)
    for i in range(samples):


        lp1=getPoint(data,i*(115/samples)+5)
        rp1=getPoint(data,240-(i*(115/samples)+5))
        w=.2
        if (lp1!=None and lp!=None):
            l=Line(lp1, lp)
            addCombineWall(l)

            wp1=Point((lp1.x+w*cp.x)/(1.0+w),(lp1.y+w*cp.y)/(1.0+w))
            removeLinesIntersecting(Line(wp1, cp))


            #wall.append(Line(lp,lp1))

            #addWall(Line(lp,lp1))

        if (rp1!=None and rp!=None):
            l=Line(rp1, rp)
            #print(LineFunc(l))
            f=LineFunc(l)
            #print(f)
            print(f.m,"x+",f.b)
            addCombineWall(l)
            wp1=Point((rp1.x+w*cp.x)/(1.0+w),(rp1.y+w*cp.y)/(1.0+w))
            removeLinesIntersecting(Line(wp1, cp))
            #wall.append(Line(rp,rp1))
            #addWall(Line(rp,rp1))

        lp=lp1
        rp=rp1
    #cleanPoints()

    if (time.time() > lt + 2):
        lt = time.time()
        #avgPoints()
        #updateWalls()
        #combineSimilarLines()
        oldLocs.append(Point(x / scale, y / scale))
        if(len(oldLocs)>2):
            removeLinesIntersecting(Line(oldLocs[len(oldLocs)-1],oldLocs[len(oldLocs)-2]))



#def tryCreatingLine(m,b):
    


def addCombineWall(l):
    for l1 in wall:
        if (doLinesIntersect(l,l1)):
            combineLines(l1,l)
            return
    wall.append(l)

    pass

def addWall(l1):
    global lscale,scale

    maxdist=1.0*lscale/scale
    for w in wall:
        if (l1 != w):
            sim = getLineOrientSimilarity(w, l1)
            if (sim > .9 and sim < 1.01):
                dist = getDistBetweenLines(w, l1)
                if (maxdist > dist):
                    wall.append(combineLines(w, l1))
                    wall.remove(w)
                    return
    wall.append(l1)

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
    print ("combineing took ",t," and there are now ",len(wall)," walls")

def removeLinesIntersecting(l1):
    for w in wall:
        if(doLinesIntersect(l1,w)):
            #print (l1," intersects with ",w)
            wall.remove(w)


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

def getLineOrientSimilarity(l1,l2):
    m1=np.math.atan((l1.p2.getY()-l1.p1.getY())/(l1.p2.getX()-l1.p1.getX()))
    m2=np.math.atan((l2.p2.getY()-l2.p1.getY())/(l2.p2.getX()-l2.p1.getX()))

    if (m1>0!=m2>0):
        return 0
    if (m1>m2):
        return (m2/m1)
    else:
        return (m1/m2)



def updateWalls():
    i=0
    global points,lscale,scale

    for p in points:
        c=getClosestPoint(p)
        if (c!=None):
            if(distBetween(p,c)<2*lscale/scale):
                wall.append( Line(p, c))
                points.remove(c)
                points.remove(p)
    points=points[len(points)/2:len(points)]
    points=points[len(points)/2:len(points)]
    points=[]
    #test

def getLineOrient(l):
    dx=l.p2.getX()-l.p1.getX()
    dy=l.p2.getY()-l.p1.getY()
    o=np.math.atan2(dy,dx)
    return o

def getClosestPoint(p1):
    x=p1.getX()
    y=p1.getY()
    closest=None
    closest_dist=10000
    for p in points:
        if (p!=p1):
            dx=p.getX()-x
            dy=p.getY()-y
            dist=np.math.sqrt(dx*dx+dy*dy)
            if(dist<closest_dist):
                closest=p
                #print ("new closest dist is ",closest_dist)
                closest_dist=dist

    global scale, lscale
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
    md=.45*lscale/scale #md = max distance
    mindist=.8*lscale/scale
    for p in points:
        x=p.getX()
        y=p.getY()
        #print (x,y)
        for p1 in points:
            if (p1!=p):
                x1 = p1.getX()
                y1 = p1.getY()
                if(abs(x1-x)<md and abs(y1-y)<md):
                    points.remove(p1)
        close=getClosestPoint(p)

        if(close!=None):
            dist=distBetween(p,close)
            if (dist>mindist):
                points.remove(p)



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
        global x,y,points,points
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


#def tryMakeLine()

class LineFunc():

    m = None  # slope
    b = None  # y intercept
    orient=None

    def __init__(self,l1):
        self.m=(l1.p2.getX() - l1.p1.getX()) / ((l1.p2.getY() - l1.p1.getY())+.000001)
        self.b=l1.p1.getY()-(l1.p1.getX()*self.m)
        self.orient=np.math.atan(1.0/self.m)

    def f(self,x):
        return (self.m*x)+self.b

    def getLineSim(self, l1, maxDistDif, maxOrientDif):
        global scale,lscale
        func=LineFunc(l1)
        orientsim=1-(abs(self.orient-func.orient)/3.14)
        dy1=self.f(l1.p1.getX())-l1.p1.getY()
        dy2=self.f(l1.p2.getX())-l1.p2.getY()
        dy=abs((dy1+dy2)/2.0)
        #if ()


    
    def __repr__(self):
        return "f(x)=".format(str(self.m),"x+",str(self.b))

