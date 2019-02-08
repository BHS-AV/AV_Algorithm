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
points=[]
wall=[]

def update(dir):
    global win,front,c,x,y,orient, oldLocs
    d1=dir/180.0*3.14
    orient=d1

def render():
    global win,front,c,x,y,orient, oldLocs
    c = Circle(Point(x/scale,y/scale), 10)
    c.setFill("black")
    front= Circle(Point(x/scale+(10*np.math.cos(orient)),y/scale+(10*np.math.sin(orient))),5)
    front.setFill("red")
    clear(win)
    for pp in oldLocs:
        p=Circle(pp, 3)
        p.setFill("orange")
        p.draw(win)

    for pp in points:
        p=Circle(pp, 3)
        p.setFill("blue")
        p.draw(win)
    for l in wall:
        l.draw(win)
    c.draw(win)
    front.draw(win)


def scanWalls(data):
    global orient
    if (orient==0):return
    samples=20
    for i in range(samples):
        addPoint(data,i*(115/samples)+5)
        addPoint(data,240-(i*(115/samples)+5))

    cleanPoints()

    global oldLocs, lt

    if (time.time() > lt + 2):
        lt = time.time()
        #avgPoints()
        updateWalls()
        oldLocs.append(Point(x / scale, y / scale))


def updateWalls():
    i=0
    global points,lscale,scale

    for p in points:
        c=getClosestPoint(p)
        if (c!=None):
            if(distBetween(p,c)<2*lscale/scale):
                wall.append( Line(p, c))
    #points=points[len(points)/2:len(points)]
    #points=points[len(points)/2:len(points)]
    #points=[]


def getClosestPoint(p1=Point(0,0)):
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
                closest_dist=dist

    global scale, lscale
    #if (dist>6*lscale/scale):
    #    return None

    return closest


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


def addPoint(data, dir):
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
        points.append(p)


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