import numpy as np
import math
import heapq


class Node:
    def __init__(self, xind, yind, yawind, direction, x, y,
                 yaw, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind

class Para0:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, reso, motion):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = xw
        self.yw = yw
        self.reso = reso  # resolution of grid world
        self.motion = motion  # motion set

class Para:
    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw,
                 xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.xyreso = xyreso
        self.yawreso = yawreso
        self.ox = ox
        self.oy = oy
        self.kdtree = kdtree

def get_motion():
    motion = [[-1, 0], [-1, 1], [0, 1], [1, 1],
              [1, 0], [1, -1], [0, -1], [-1, -1]]

    return motion

def calc_obsmap(ox, oy, rr, P):
    obsmap = [[False for _ in range(P.yw)] for _ in range(P.xw)]

    for x in range(P.xw):
        xx = x + P.minx
        for y in range(P.yw):
            yy = y + P.miny
            for oxx, oyy in zip(ox, oy):
                if math.hypot(oxx - xx, oyy - yy) <= rr / P.reso:
                    obsmap[x][y] = True
                    break

    return obsmap

def calc_parameters_arxiv(ox, oy, rr, reso):
    minx, miny = round(min(ox)), round(min(oy))
    maxx, maxy = round(max(ox)), round(max(oy))
    xw, yw = maxx - minx, maxy - miny

    motion = get_motion()
    P = Para0(minx, miny, maxx, maxy, xw, yw, reso, motion)
    obsmap = calc_obsmap(ox, oy, rr, P)

    return P, obsmap


def calc_parameters(ox, oy, xyreso, yawreso, kdtree):
    minx = round(min(ox) / xyreso)
    miny = round(min(oy) / xyreso)
    maxx = round(max(ox) / xyreso)
    maxy = round(max(oy) / xyreso)

    xw, yw = maxx - minx, maxy - miny

    PI = np.pi

    minyaw = round(-PI / yawreso) - 1
    maxyaw = round(PI / yawreso)
    yaww = maxyaw - minyaw

    return Para(minx, miny, minyaw, maxx, maxy, maxyaw,
                xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree)


def my_design_obstacles_small(reso):
    ox, oy = [], []

    # the simple obstacled environment

    for i in np.arange(-4.0,3.5,reso):
        ox.append(i)
        oy.append(1.5)

    for i in np.arange(10.0,16.0,reso):
        ox.append(i)
        oy.append(1.5)
    
    for i in np.arange(-4.0,8.0,reso):
        ox.append(i)
        oy.append(-2.0)

    for i in np.arange(1.5,4.5,reso):
        ox.append(3.5)
        oy.append(i)

    for i in np.arange(1.5,4.5,reso):
        ox.append(10.0)
        oy.append(i)

    for i in np.arange(3.5,10.0,reso):
        ox.append(i)
        oy.append(4.5)

    for i in np.arange(1.5,5.5,reso):
        ox.append(-4.0)
        oy.append(i)

    for i in np.arange(1.5,5.5,reso):
        ox.append(16.0)
        oy.append(i)

    for i in np.arange(-3.0,-2.0,reso):
        ox.append(-4.0)
        oy.append(i)

    for i in np.arange(-3.0,-2.0,reso):
        ox.append(8.0)
        oy.append(i)

    for i in np.arange(-4.0,8.0,reso):
        ox.append(i)
        oy.append(-3.0)

    for i in np.arange(-4.0,16.0,reso):
        ox.append(i)
        oy.append(5.5)


    return ox,oy


def my_design_obstacles_large(reso):
    ox, oy = [], []


    # walls around
    for i in np.arange(-25.0,25.0,reso):
        ox.append(i)
        oy.append(25.0)

    for i in np.arange(-25.0,25.0,reso):
        ox.append(i)
        oy.append(-25.0)

    for i in np.arange(-25.0,25.0,reso):
        ox.append(25.0)
        oy.append(i)
    
    for i in np.arange(-25.0,25.0,reso):
        ox.append(-25.0)
        oy.append(i)

    # obs1
    for i in np.arange(1.0,5.0,reso):
        ox.append(0.0)
        oy.append(i)

    for i in np.arange(1.0,5.0,reso):
        ox.append(-1.0)
        oy.append(i)

    for i in np.arange(-1.0,0.0,reso):
        ox.append(i)
        oy.append(1.0)

    for i in np.arange(-1.0,0.0,reso):
        ox.append(i)
        oy.append(5.0)

    # obs2
    for i in np.arange(-5.0,-1.0,reso):
        ox.append(0.0)
        oy.append(i)

    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-1.0)
        oy.append(i)

    for i in np.arange(-1.0,0.0,reso):
        ox.append(i)
        oy.append(-1.0)

    for i in np.arange(-1.0,0.0,reso):
        ox.append(i)
        oy.append(-5.0)

    # obs3
    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-2.5)
        oy.append(i)

    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-3.5)
        oy.append(i)

    for i in np.arange(-3.5,-2.5,reso):
        ox.append(i)
        oy.append(-1.0)

    for i in np.arange(-3.5,-2.5,reso):
        ox.append(i)
        oy.append(-5.0)

    # obs4
    for i in np.arange(1.0,5.0,reso):
        ox.append(-7.0)
        oy.append(i)

    for i in np.arange(1.0,5.0,reso):
        ox.append(-8.0)
        oy.append(i)

    for i in np.arange(-8.0,-7.0,reso):
        ox.append(i)
        oy.append(1.0)

    for i in np.arange(-8.0,-7.0,reso):
        ox.append(i)
        oy.append(5.0)

    # obs5
    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-7.5)
        oy.append(i)

    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-8.5)
        oy.append(i)

    for i in np.arange(-8.5,-7.5,reso):
        ox.append(i)
        oy.append(-1.0)

    for i in np.arange(-8.5,-7.5,reso):
        ox.append(i)
        oy.append(-5.0)

    # obs6
    for i in np.arange(-4.0,-0.5,reso):
        ox.append(-9.5)
        oy.append(i)

    for i in np.arange(-4.0,-0.5,reso):
        ox.append(-10.5)
        oy.append(i)

    for i in np.arange(-10.5,-9.5,reso):
        ox.append(i)
        oy.append(-0.5)

    for i in np.arange(-10.5,-9.5,reso):
        ox.append(i)
        oy.append(-4.0)

    # obs7
    for i in np.arange(0.5,4.5,reso):
        ox.append(-12.0)
        oy.append(i)

    for i in np.arange(0.5,4.5,reso):
        ox.append(-13.0)
        oy.append(i)

    for i in np.arange(-13.0,-12.0,reso):
        ox.append(i)
        oy.append(0.5)

    for i in np.arange(-13.0,-12.0,reso):
        ox.append(i)
        oy.append(4.5)

    # obs8
    for i in np.arange(0.5,4.5,reso):
        ox.append(-16.0)
        oy.append(i)

    for i in np.arange(0.5,4.5,reso):
        ox.append(-17.0)
        oy.append(i)

    for i in np.arange(-17.0,-16.0,reso):
        ox.append(i)
        oy.append(0.5)

    for i in np.arange(-17.0,-16.0,reso):
        ox.append(i)
        oy.append(4.5)
    
    # obs9
    for i in np.arange(-5.0,-0.5,reso):
        ox.append(-19.5)
        oy.append(i)

    for i in np.arange(-5.0,-0.5,reso):
        ox.append(-18.5)
        oy.append(i)

    for i in np.arange(-19.5,-18.5,reso):
        ox.append(i)
        oy.append(-0.5)

    for i in np.arange(-19.5,-18.5,reso):
        ox.append(i)
        oy.append(-5.0)

    # obs10
    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-22.0)
        oy.append(i)

    for i in np.arange(-5.0,-1.0,reso):
        ox.append(-21.0)
        oy.append(i)

    for i in np.arange(-22.0,-21.0,reso):
        ox.append(i)
        oy.append(-1.0)

    for i in np.arange(-22.0,-21.0,reso):
        ox.append(i)
        oy.append(-5.0)

    # obs11
    for i in np.arange(1.0,5.0,reso):
        ox.append(-22.0)
        oy.append(i)

    for i in np.arange(1.0,5.0,reso):
        ox.append(-21.0)
        oy.append(i)

    for i in np.arange(-22.0,-21.0,reso):
        ox.append(i)
        oy.append(1.0)

    for i in np.arange(-22.0,-21.0,reso):
        ox.append(i)
        oy.append(5.0)

    # obs12
    for i in np.arange(1.0,5.0,reso):
        ox.append(3.5)
        oy.append(i)

    for i in np.arange(1.0,5.0,reso):
        ox.append(4.5)
        oy.append(i)

    for i in np.arange(3.5,4.5,reso):
        ox.append(i)
        oy.append(1.0)

    for i in np.arange(3.5,4.5,reso):
        ox.append(i)
        oy.append(5.0)

    # obs13
    for i in np.arange(-5.0,6.0,reso):
        ox.append(5.35)
        oy.append(i)

    for i in np.arange(-5.0,6.0,reso):
        ox.append(5.85)
        oy.append(i)

    # obs14
    for i in np.arange(13.0,20.0,reso):
        ox.append(17.0)
        oy.append(i)

    for i in np.arange(13.0,20.0,reso):
        ox.append(24.0)
        oy.append(i)

    for i in np.arange(17.0,24.0,reso):
        ox.append(i)
        oy.append(13.0)

    for i in np.arange(17.0,24.0,reso):
        ox.append(i)
        oy.append(20.0)
    
    # obs15
    for i in np.arange(0.0,3.0,reso):
        ox.append(16.0)
        oy.append(i)

    for i in np.arange(0.0,3.0,reso):
        ox.append(24.0)
        oy.append(i)

    for i in np.arange(16.0,21.0,reso):
        ox.append(i)
        oy.append(0.0)

    for i in np.arange(16.0,24.0,reso):
        ox.append(i)
        oy.append(3.0)

    # obs16
    for i in np.arange(-18.0,0.0,reso):
        ox.append(21.0)
        oy.append(i)

    for i in np.arange(-18.0,0.0,reso):
        ox.append(24.0)
        oy.append(i)

    for i in np.arange(21.0,24.0,reso):
        ox.append(i)
        oy.append(-18.0)

    # obs17
    for i in np.arange(-5.0,-4.0,reso):
        ox.append(17.5)
        oy.append(i)

    for i in np.arange(-5.0,-4.0,reso):
        ox.append(20.5)
        oy.append(i)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-5.0)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-4.0)
    
    # obs18
    for i in np.arange(-6.5,-5.5,reso):
        ox.append(17.5)
        oy.append(i)

    for i in np.arange(-6.5,-5.5,reso):
        ox.append(20.5)
        oy.append(i)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-6.5)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-5.5)

    # obs19
    for i in np.arange(-12.0,-11.0,reso):
        ox.append(17.5)
        oy.append(i)

    for i in np.arange(-12.0,-11.0,reso):
        ox.append(20.5)
        oy.append(i)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-11.0)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-12.0)
    
    # obs20
    for i in np.arange(-14.5,-13.5,reso):
        ox.append(17.5)
        oy.append(i)

    for i in np.arange(-14.5,-13.5,reso):
        ox.append(20.5)
        oy.append(i)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-14.5)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-13.5)

    # obs21
    for i in np.arange(-17.0,-16.0,reso):
        ox.append(17.5)
        oy.append(i)

    for i in np.arange(-17.0,-16.0,reso):
        ox.append(20.5)
        oy.append(i)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-17.0)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(-16.0)

    # obs22
    for i in np.arange(15.5,19.5,reso):
        ox.append(8.0)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(9.0)
        oy.append(i)

    for i in np.arange(8.0,9.0,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(17.5,20.5,reso):
        ox.append(i)
        oy.append(19.5)

    # obs23
    for i in np.arange(15.5,19.5,reso):
        ox.append(5.5)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(6.5)
        oy.append(i)

    for i in np.arange(5.5,6.5,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(5.5,6.5,reso):
        ox.append(i)
        oy.append(19.5)

    # obs24
    for i in np.arange(15.5,19.5,reso):
        ox.append(0.5)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(1.5)
        oy.append(i)

    for i in np.arange(0.5,1.5,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(0.5,1.5,reso):
        ox.append(i)
        oy.append(19.5)

    # obs25
    for i in np.arange(15.5,19.5,reso):
        ox.append(-10.5)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(-9.5)
        oy.append(i)

    for i in np.arange(-10.5,-9.5,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(-10.5,-9.5,reso):
        ox.append(i)
        oy.append(19.5)


    # obs26
    for i in np.arange(15.5,19.5,reso):
        ox.append(-18.5)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(-17.5)
        oy.append(i)

    for i in np.arange(-18.5,-17.5,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(-18.5,-17.5,reso):
        ox.append(i)
        oy.append(19.5)

    # obs27
    for i in np.arange(15.5,19.5,reso):
        ox.append(-21.0)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(-20.0)
        oy.append(i)

    for i in np.arange(-21.0,-20.0,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(-21.0,-20.0,reso):
        ox.append(i)
        oy.append(19.5)

    # obs28
    for i in np.arange(15.5,19.5,reso):
        ox.append(-21.5)
        oy.append(i)

    for i in np.arange(15.5,19.5,reso):
        ox.append(-22.5)
        oy.append(i)

    for i in np.arange(-22.5,-21.5,reso):
        ox.append(i)
        oy.append(15.5)

    for i in np.arange(-22.5,-21.5,reso):
        ox.append(i)
        oy.append(19.5)

    # obs29
    for i in np.arange(-2.0,8.0,reso):
        ox.append(i)
        oy.append(-11.5)

    for i in np.arange(-2.0,8.0,reso):
        ox.append(i)
        oy.append(-12.5)

    # obs30
    for i in np.arange(-2.0,4.0,reso):
        ox.append(i)
        oy.append(-16.5)

    for i in np.arange(-2.0,4.0,reso):
        ox.append(i)
        oy.append(-17.5)

    # obs31
    for i in np.arange(-18.0,-11.0,reso):
        ox.append(-8.5)
        oy.append(i)

    # obs32
    for i in np.arange(-18.0,-13.0,reso):
        ox.append(-11.0)
        oy.append(i)
    for i in np.arange(-18.0,-13.0,reso):
        ox.append(-10.0)
        oy.append(i)

    # obs33
    for i in np.arange(-18.0,-13.0,reso):
        ox.append(-13.5)
        oy.append(i)
    for i in np.arange(-18.0,-13.0,reso):
        ox.append(-12.5)
        oy.append(i)



    return ox,oy



# def isObstacled(x,y,obsmap,P,reso):
#     '''
#     determines whether a location is obstacled (returns True) or not (returns False).
#     Input:
#         - x,y   : coordinate location (unit: meter)
#         - obsmap: predefined obstacle map
#         - reso  : resolution of map
#     '''
#     x = x/reso
#     y = y/reso

#     idx_x = int(x-P.minx)
#     idx_y = int(y-P.miny)
    
#     # print(idx_x,idx_y)

#     haveObstacle = obsmap[idx_x][idx_y]

#     return haveObstacle


# if __name__=='__main__':

#     reso = 0.1
#     ox,oy = get_env_simple(reso)

#     rr = 0.1 # car radius

#     ox = [x / reso for x in ox]
#     oy = [y / reso for y in oy]

#     P, obsmap = calc_parameters(ox, oy, rr, reso)

#     print("---Obstacle map calculate finished---")
#     print("assume a 0.1m radius of detection redius")
#     print("x: ",1.59,"y: ",1.6,": ",isObstacled(1.59, 1.6, obsmap, P, reso))
#     print("x: ",1.6,"y: ",1.6,": ",isObstacled(1.6, 1.6, obsmap, P, reso))
#     print("x: ",2.75,"y: ",1.75,": ",isObstacled(2.75, 1.75, obsmap, P, reso))
    