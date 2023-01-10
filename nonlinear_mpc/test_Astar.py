import matplotlib.pyplot as plt
import numpy as np
import time, os
import path
from acados_settings import *
import bicycle_model
import mpc_plot
import obstacle
a = np.load("ref_simple1.npy")
x = a[:,0]
y = a[:,1]
yaw = a[:,2]
direction = a[:,3]
hybridastar_path = path.PATH(x,y,yaw,0)
speed_profile = path.get_velprofile(hybridastar_path,1.,0.1)
# mpc_plot.plot_velprof(x,y,yaw,speed_profile)
# plt.show()
test_param = test_param ={"N":6,   # Predict Horizon
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
init_state = np.array([state.x,state.y,state.v,state.yaw,state.delta])
constraint, model, acados_solver = acados_settings(Tf, N, init_state)
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)
for i in range(Nsim):
    # update reference
    plt.cla()
    ref,ind,dir = path.get_reftraj(state,hybridastar_path,speed_profile,test_param)
    # print("ref",ref)

    x0 = np.array([state.x,state.y,state.v,state.yaw,state.delta])
    # print("x0",x0)
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)

    for j in range(N):

        yref = np.zeros(7)
        yref[:5] = np.array(ref[:, j])
        acados_solver.set(j, "yref", yref)
    # solve
    acados_solver.set(N, "yref", np.array(ref[:, N]))
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))
    x_pred = [acados_solver.get(j, "x") for j in range(N + 1)]
    u = [acados_solver.get(j, "u") for j in range(N)]

    print("x_pred", x_pred)
    state.state_update(u[0][0], u[0][1], test_param)

    mpc_plot.plot_mpc(hybridastar_path.cx, hybridastar_path.cy, hybridastar_path.cyaw, ref, x_pred, u, speed_profile)

    mpc_plot.draw_car(state.x, state.y, state.yaw, state.delta)
    plt.xlim(0, 12)
    plt.ylim(-6, 6)
    plt.title("Linear MPC, " + "v = " + str(state.v) + "\n delta = " + str(state.delta))
    # if status != 0:
    #     plt.pause(10)
    # plt.show()
    plt.pause(0.1)