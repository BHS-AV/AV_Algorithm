import numpy as np
from graphics import *

#from Scripts.graphics import Circle, Point

win = GraphWin("My Circle", 1000, 1000)

#if __name__=="__main__":
    #global win
    #win = GraphWin("My Circle", 100, 100)
c=None
front=None
scale=2
x=win.getWidth()/2*scale
y=win.getHeight()/2*scale
orient=0
lt=time.time()

oldLocs=[Point(x / scale, y / scale)]
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
        p.setFill("blue")
        p.draw(win)
    for pp in points:
        p=Circle(pp, 3)
        p.setFill("green")
        p.draw(win)
    for l in wall:
        l.draw(win)
    c.draw(win)
    front.draw(win)

def updateWalls():
    i=0
    global points
    for p in range(len(points)):
        if (i>1):
            l=Line(points[p],points[p-2])
            wall.append(l)
        i=i+1
    points=[]


def cleanPoints():
    global points
    for p in points:
        x=p.getX()
        y=p.getY()
        #print (x,y)
        for p1 in points:
            if (p1!=p):
                x1 = p1.getX()
                y1 = p1.getY()
                if(abs(x1-x)<4 and abs(y1-y)<4):
                    points.remove(p1)

def scanWalls(data):
    for i in range(10):
        addPoint(data,i*20+20)

def addPoint(data, dir):
    global scale
    lscale=20.0
    #r=get_dist(data,dir)*lscale/scale
    d=get_data_array(data, dir-10,dir+10)
    if (d.max()<8):
        global x,y,points
        d1=(dir-135)/180.0*3.14
        r=d.mean()*lscale/scale
        p=Point(x / scale + (r * np.math.cos(d1)), y / scale + (r * np.math.sin(d1)))
        points.append(p)

def get_dist(data, a=0):
    PPD = 4.5  # points per degree
    return get_data_array(data,a-10,a+10).mean()
    #return data[int(a * PPD)]

def get_data_array(data, a=0, b=240):
    PPD = 4.5  # points per degree
    data = data[int(a * PPD):int(b * PPD)]
    arr = np.array(data)
    return arr

def move(amt):
    global x,y,orient,oldLocs, lt
    if (time.time()>lt+5):
        lt=time.time()
        #updateWalls()
        cleanPoints()
        oldLocs.append(Point(x / scale, y / scale))
    x=x+ (amt * np.math.cos(orient))
    y=y + (amt * np.math.sin(orient))

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()