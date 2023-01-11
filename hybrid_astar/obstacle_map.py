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


# class C:  # Parameter config
#     PI = math.pi

#     XY_RESO = 0.1  # [m]
#     # XY_RESO = 0.1 # yongxi
#     YAW_RESO = np.deg2rad(5.0)  # [rad]
#     # YAW_RESO = np.deg2rad(5.0)  # yongxi [rad]
#     MOVE_STEP = 0.04  # [m] path interporate resolution
#     # MOVE_STEP = 0.04  # [m] path interporate resolution 
#     # N_STEER = 20.0  # steer command number
#     N_STEER = 2.0  # steer command number
#     COLLISION_CHECK_STEP = 3  # skip number for collision check
#     EXTEND_BOUND = 1  # collision check range extended

#     RECTIF = 10.0
#     GEAR_COST = 100.0 * RECTIF # switch back penalty cost
#     BACKWARD_COST = 5.0 * RECTIF # backward penalty cost
#     STEER_CHANGE_COST = 5.0 * RECTIF # steer angle change penalty cost
#     STEER_ANGLE_COST = 1.0 * RECTIF # steer angle penalty cost
#     H_COST = 15.0 * RECTIF # Heuristic cost penalty cost

#     RESIZE = 1.0
#     RF = 3.45 * RESIZE
#     RB = 0.6  * RESIZE
#     W = 1.8 * RESIZE # chasis is 1.7526
#     WD = 1.534 * RESIZE
#     WB = 2.85   * RESIZE # [m] Wheel base
#     TR = 0.31265 * RESIZE
#     TW = 0.4 * RESIZE
#     MAX_STEER = 0.6 # [rad] maximum steering angle


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
    for i in np.arange(4.0,10.0,reso):
        for j in np.arange(1.0,3.0,reso):
            ox.append(i)
            oy.append(j)

    # obstacle 4
    for i in np.arange(-4.0,10.0,reso):
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
    