import matplotlib.pyplot as plt
import numpy as np
import time, os
import path
from acados_settings import *
import bicycle_model
import mpc_plot
import obstacle
from mpc_controller import mpc_controller
a = np.load("ref_simple1.npy")
x,y, yaw, direction = a[:,0],a[:,1],a[:,2],a[:,3]
hybridastar_path = path.PATH(x,y,yaw,0)
speed_profile = path.get_velprofile(hybridastar_path,1.,0.1)
test_param = test_param ={
        "T":60,
        "N":6,   # Predict Horizon
        "Tf":2.4,  # dim of state space
        "dt":0.4, #time step
        "d_dist":0.04,#todo:distance between nodes
        "N_IND":10, # search index number
        "lr":1.425,
        "L":2.85,    # lr+lf
        "disc_offset":1,
        "radius":1.455,
        "start_vel":0.25,
        "approximate_acc" :0.2,
        "max_acc":1.0,
        "max_steer_vel":0.6,
        }
T=60
Tf = test_param["Tf"]  # prediction horizon
N = test_param["N"]  # number of discretization steps
disc_offset = test_param["disc_offset"]
radius = test_param["radius"]
state = bicycle_model.ROBOT_STATE(x=1.0125, y=0, yaw=0, v=0.0)
Nsim = int(T * N / Tf)
if __name__ == '__main__':
    # 1. initialize
    mpc = mpc_controller(hybridastar_path, test_param, state,speed_profile)
    # 2. simulate
    for i in range(Nsim):
        u,ref,x_pred = mpc.control()
        mpc.state.state_update(u[0], u[1], test_param)
        mpc.visualize(ref,x_pred)
        time.sleep(0.1)

