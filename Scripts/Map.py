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
rpoints=[]
lpoints=[]
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
    for pp in rpoints:
        p=Circle(pp, 3)
        p.setFill("green")
        p.draw(win)
    for pp in lpoints:
        p=Circle(pp, 3)
        p.setFill("blue")
        p.draw(win)
    for l in wall:
        l.draw(win)
    c.draw(win)
    front.draw(win)

def updateWalls():
    i=0
    global lpoints, rpoints
    for p in range(len(lpoints)):
        if (i>1):
            l=Line(lpoints[p-1],lpoints[p])
            wall.append(l)
        i=i+1
    #lpoints=[lpoints[len(lpoints)-2]]
    lpoints=[]
    for p in range(len(rpoints)):
        if (i>1):
            l=Line(rpoints[p-1],rpoints[p])
            wall.append(l)
        i=i+1
    #rpoints=[rpoints[len(rpoints)-2]]
    rpoints=[]

def avgPoints():
    i=0
    global rpoints, lpoints
    for p in range(len(rpoints)):
        if (i>1 and i+1<len(rpoints)):
            x=(rpoints[p-1].getX()+rpoints[p].getX()+rpoints[p+1].getX())/3
            y=(rpoints[p-1].getY()+rpoints[p].getY()+rpoints[p+1].getY())/3
            rpoints[p]= Point(x,y)
        i=i+1
    i = 0
    for p in range(len(lpoints)):
        if (i > 1 and i + 1 < len(lpoints)):
            x = (lpoints[p - 1].getX() + lpoints[p].getX() + lpoints[p + 1].getX())/3
            y = (lpoints[p - 1].getY() + lpoints[p].getY() + lpoints[p + 1].getY())/3
            lpoints[p] = Point(x, y)

        i = i + 1


def cleanPoints():
    global rpoints,lpoints,lscale, scale
    md=lscale/scale #md = max distance
    for p in rpoints:
        x=p.getX()
        y=p.getY()
        #print (x,y)
        for p1 in rpoints:
            if (p1!=p):
                x1 = p1.getX()
                y1 = p1.getY()
                if(abs(x1-x)<md and abs(y1-y)<md):
                    rpoints.remove(p1)
    for p in lpoints:
        x = p.getX()
        y = p.getY()
        # print (x,y)
        for p1 in lpoints:
            if (p1 != p):
                x1 = p1.getX()
                y1 = p1.getY()
                if (abs(x1 - x) < md and abs(y1 - y) < md):
                    lpoints.remove(p1)

def scanWalls(data):
    global orient
    if (orient==0):return
    samples=10
    for i in range(samples):
        addPoint(data,i*(115/samples)+5)
        addPoint(data,240-(i*(115/samples)+5))

    global oldLocs, lt

    if (time.time() > lt + 1):
        lt = time.time()
        cleanPoints()
        # updateWalls()
        #avgPoints()
        oldLocs.append(Point(x / scale, y / scale))

def addPoint(data, dir):
    global scale, orient,lscale
    r=get_dist(data,dir)
    #d=get_data_array(data, dir-3,dir+3)
    #dir=240-dir
    if (r<8):
        global x,y,rpoints,lpoints
        r=r*lscale/scale
        d1=((((dir)-120)*3.14)/180.0)+orient
        #r=d.mean()*lscale/scale
        p=Point(x / scale + (r * np.math.cos(d1)), y / scale + (r * np.math.sin(d1)))
        if(dir<120):
            rpoints.append(p)
        else:
            lpoints.append(p)

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