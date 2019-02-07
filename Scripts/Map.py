import numpy as np
from graphics import *

#from Scripts.graphics import Circle, Point

win = GraphWin("My Circle", 300, 300)

#if __name__=="__main__":
    #global win
    #win = GraphWin("My Circle", 100, 100)
c=None
front=None
scale=4.0
x=win.getWidth()/2*scale
y=win.getHeight()/2*scale
orient=0
lt=time.time()

points=[Point(x/scale,y/scale)]

def update(dir):
    global win,front,c,x,y,orient, points
    c = Circle(Point(x/scale,y/scale), 10)
    c.setFill("black")
    d1=dir/180.0*3.14
    orient=d1
    front= Circle(Point(x/scale+(10*np.math.cos(d1)),y/scale+(10*np.math.sin(d1))),5)
    front.setFill("red")

    clear(win)

    for pp in points:
        p=Circle(pp, 3)
        p.setFill("blue")
        p.draw(win)


    c.draw(win)
    front.draw(win)


def move(amt):
    global x,y,orient,points, lt
    if (time.time()>lt+1):
        lt=time.time()
        points.append(Point(x/scale,y/scale))
    x=x+ (amt * np.math.cos(orient))
    y=y + (amt * np.math.sin(orient))

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()