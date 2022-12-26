import numpy as np
import math
import heapq


class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x  # x position of node
        self.y = y  # y position of node
        self.cost = cost  # g cost of node
        self.pind = pind  # parent index of node

class Para:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, reso, motion):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = xw
        self.yw = yw
        self.reso = reso  # resolution of grid world
        self.motion = motion  # motion set

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

def calc_parameters(ox, oy, rr, reso):
    minx, miny = round(min(ox)), round(min(oy))
    maxx, maxy = round(max(ox)), round(max(oy))
    xw, yw = maxx - minx, maxy - miny

    motion = get_motion()
    P = Para(minx, miny, maxx, maxy, xw, yw, reso, motion)
    obsmap = calc_obsmap(ox, oy, rr, P)

    return P, obsmap

def get_env_simple(reso):

    ox=list()
    oy=list()

    # obstacle 1
    for i in np.arange(-4.0,1.5,reso):
        for j in np.arange(1.0,3.0,reso):
            ox.append(i)
            oy.append(j)
    
    # obstacle 2
    for i in np.arange(1.5,4.0,reso):
        for j in np.arange(2.5,3.0,reso):
            ox.append(i)
            oy.append(j)

    # obstacle 3 
    for i in np.arange(4.0,5.0,reso):
        for j in np.arange(1.0,3.0,reso):
            ox.append(i)
            oy.append(j)

    # obstacle 4
    for i in np.arange(-4.0,5.0,reso):
        for j in np.arange(-2.0,-1.0,reso):
            ox.append(i)
            oy.append(j)

    return ox,oy

def isObstacled(x,y,obsmap,P,reso):
    '''
    determines whether a location is obstacled (returns True) or not (returns False).
    Input:
        - x,y   : coordinate location (unit: meter)
        - obsmap: predefined obstacle map
        - reso  : resolution of map
    '''
    x = x/reso
    y = y/reso

    idx_x = int(x-P.minx)
    idx_y = int(y-P.miny)
    
    # print(idx_x,idx_y)

    haveObstacle = obsmap[idx_x][idx_y]

    return haveObstacle


if __name__=='__main__':

    reso = 0.1
    ox,oy = get_env_simple(reso)

    rr = 0.1 # car radius

    ox = [x / reso for x in ox]
    oy = [y / reso for y in oy]

    P, obsmap = calc_parameters(ox, oy, rr, reso)

    print("---Obstacle map calculate finished---")
    print("assume a 0.1m radius of detection redius")
    print("x: ",1.59,"y: ",1.6,": ",isObstacled(1.59, 1.6, obsmap, P, reso))
    print("x: ",1.6,"y: ",1.6,": ",isObstacled(1.6, 1.6, obsmap, P, reso))
    print("x: ",2.75,"y: ",1.75,": ",isObstacled(2.75, 1.75, obsmap, P, reso))
    