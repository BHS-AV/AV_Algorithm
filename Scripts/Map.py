import numpy as np
from graphics import *

#from Scripts.graphics import Circle, Point

win = GraphWin("My Circle", 300, 300)

#if __name__=="__main__":
    #global win
    #win = GraphWin("My Circle", 100, 100)
c=None
front=None

def update(dir):
    global win,front,c

    c = Circle(Point(50,50), 10)
    c.setFill("black")
    d1=dir/180.0*3.14

    front= Circle(Point(50+(10*np.math.cos(d1)),50+(10*np.math.sin(d1))),5)
    front.setFill("red")

    clear(win)


    c.draw(win)
    front.draw(win)

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()