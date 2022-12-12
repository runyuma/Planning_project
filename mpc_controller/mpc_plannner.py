import cvxpy
import numpy as np
import matplotlib.pyplot as plt
param ={"N":6,   # Predict Horizon
        "Nx":4,  # dim of state space
        "dt":0.2, #time step
        "d_dist":1,#todo:distance between nodes
        "N_IND":10, # search index number
        "lr":1.25,
        "L":2.5,    # lr+lf
        "max_vel":10,
        "min_vel":-5,
        "max_steer":np.deg2rad(20),
        "min_steer":-np.deg2rad(20),
        "max_acc":1,
        "min_acc":-1,
        }
class MPC_PLANNER():
    def __init__(self):
        pass
class ROBOT_STATE():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        #todo:delta
def get_velprofile(cx, cy, cyaw, target_speed):
    """
        design appropriate speed strategy
        :param cx: x of reference path [m]
        :param cy: y of reference path [m]
        :param cyaw: yaw of reference path [m]
        :param target_speed: target speed [m/s]
        :return: speed profile
        """

    speed_profile = [target_speed] * len(cx)
    direction = [1.0 for i in range(len(cx))]  # forward

    # Set stop point
    dx = [cx[i + 1] - cx[i] for i in range(len(cx) - 1)]
    dy = [cy[i + 1] - cy[i] for i in range(len(cy) - 1)]
    dyaw = [cyaw[i + 1] - cyaw[i] for i in range(len(cyaw) - 1)]
    # decide the direction of velocity
    for i in range(len(cx) - 1):
        move_direction = np.arctan2(dy[i], dx[i])
        if dx != 0.0 and dy != 0.0:
            dangle = move_direction - cyaw[i]
            if abs(dangle)>=np.pi:
                dangle += -2* np.sign(dangle)*np.pi
            if abs(dangle) >= np.pi / 4.0:
                direction[i] = -1.0
            else:
                direction[i] = 1.0
    for i in range(len(direction)):
        if direction[i] != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
    # if change direction, set vel = 0
    for i in range(len(cx) - 1):
        if direction[i] != direction[i+1]:
            speed_profile[i+1] = 0.0
    speed_profile[0] = 0.0
    speed_profile[-1] = 0.0

    return speed_profile

def get_reftraj(robot_state,ref_path,vel_profile,param):

    Nx = param["Nx"]
    N = param["N"]
    dt = param["dt"]
    d_dist = param["d_dist"]
    z_ref = np.zeros((Nx, N + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(robot_state)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = vel_profile[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0

    for i in range(1, N + 1):
        dist_move += abs(robot_state.v) * dt
        ind_move = int(round(dist_move / d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = vel_profile[index]
        z_ref[3, i] = ref_path.cyaw[index]

    return z_ref, ind
def nonlinear_mpc_control(z_ref,initial_state,):
    #todo
    pass

