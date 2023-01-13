import matplotlib.pyplot as plt
import numpy as np
import time, os
import path
from acados_settings import *
import bicycle_model
import mpc_plot
import obstacle

# os.chdir("/home/marunyu/study/planning/Planning_project")

test_path = path.test_path
speed_profile = path.get_velprofile(test_path,3.,0.1)
test_param = path.test_param

Tf = test_param["Tf"]  # prediction horizon
N = test_param["N"]  # number of discretization steps
disc_offset = test_param["disc_offset"]
radius = test_param["radius"]
T = 80.00
state = bicycle_model.ROBOT_STATE(x=10.0, y=7.0, yaw=2.09, v=0.0)
init_state = np.array([state.x,state.y,state.v,state.yaw,state.delta])


constraint, model, acados_solver = acados_settings(Tf, N, init_state)
obstacles = []
obstacles.append(obstacle.box(45, 14.5,0, 10, 2))
obstacles.append(obstacle.box(45, 21,0, 10, 2))
# obstacles.append(obstacle.box(4.5, 21,0,1,1))
obstacles.append(obstacle.circle(4.5, 21,1))

# dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
Nsim = int(T * N / Tf)


x_rec = []
y_rec = []
yaw_rec = []
v_rec = []
t_rec = []
delta_rec = []
derdelta_rec = []
a_rec = []

state = bicycle_model.ROBOT_STATE(x=10.0, y=7.0, yaw=2.09, v=0.0)
for i in range(Nsim):
    # update reference
    ref,ind,dir = path.get_reftraj(state,test_path,speed_profile,test_param)
    plt.cla()


    x0 = np.array([state.x,state.y,state.v,state.yaw,state.delta])
    # print("x0",x0)
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    for j in range(N+1):
        hyperplanes = []
        for obs in obstacles:
            obs.plot(plt)
            h = obs.find_hyperplane(state, ref[:,j], 1.25)
            if h is not None:
                obs.draw_hyperplane(h, plt)
            hyperplanes.append(h)
            print("h:",h)
        constraint_num = len(hyperplanes)
        C = np.zeros((constraint_num*2, nx))
        D = np.zeros((constraint_num*2, nu))
        g_l = -1000*np.ones(constraint_num*2)
        g_u = 1000*np.ones(constraint_num*2)
        for i in range(constraint_num):
            if hyperplanes[i] is not None:
                h = hyperplanes[i]
                T = np.array([[1, 0, 0, -np.sin(state.yaw) * disc_offset, 0],
                              [0, 1, 0, np.cos(state.yaw) * disc_offset, 0],
                              [1, 0, 0, np.sin(state.yaw) * disc_offset, 0],
                              [0, 1, 0, -np.cos(state.yaw) * disc_offset, 0]])
                t = np.array([disc_offset * np.cos(state.yaw) + state.yaw * np.sin(state.yaw) * disc_offset,
                              disc_offset * np.sin(state.yaw) - state.yaw * np.cos(state.yaw) * disc_offset,
                              -disc_offset * np.cos(state.yaw) - state.yaw * np.sin(state.yaw) * disc_offset,
                              -disc_offset * np.sin(state.yaw) + state.yaw * np.cos(state.yaw) * disc_offset])
                L = np.array([[h[0],h[1],0,0],
                              [0,0,h[0],h[1]]])
                C[i*2:(i+1)*2,:] = L @ T
                g_u[i*2:(i+1)*2] = np.array([-h[2], -h[2]]) -L @ t
                g_l[i*2:(i+1)*2] = np.array([-np.inf, -np.inf])

        acados_solver.constraints_set(j, "C", C, api='new')
        if j < N:
            acados_solver.constraints_set(j, "D", D, api='new')
        acados_solver.constraints_set(j, "lg", g_l)
        acados_solver.constraints_set(j, "ug", g_u)
    for j in range(N):
        yref = np.zeros(7)
        yref[:5] = np.array(ref[:,j])
        acados_solver.set(j, "yref", yref)

    acados_solver.set(N, "yref", np.array(ref[:,N]))
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))
        print(C@x0-g_u)
    x_pred = [acados_solver.get(j, "x") for j in range(N+1)]
    u = [acados_solver.get(j, "u") for j in range(N)]

    print("x_pred",x_pred)
    state.state_update(u[0][0],u[0][1],test_param)

    mpc_plot.plot_mpc(test_path.cx, test_path.cy, test_path.cyaw, ref, x_pred, speed_profile)

    mpc_plot.draw_car(state.x, state.y, state.yaw, state.delta)
    plt.xlim(0, 50)
    plt.ylim(-15, 35)
    plt.title("Linear MPC, " + "v = " + str(state.v) + "\n delta = " + str(state.delta))
    # if status != 0:
    #     plt.pause(10)
    plt.pause(0.001)
    x_rec.append([state.x, ref[0][0]])
    y_rec.append([state.y, ref[1][0]])
    v_rec.append([state.v, ref[2][0]])
    delta_rec.append([state.delta, ref[4][0]])
    yaw_rec.append([state.yaw, ref[3][0]])
    derdelta_rec.append([u[0][1]])
    # plt.show()

# plt.cla()
# plt.subplot(2,2,1)
# plt.plot(np.array(x_rec)[:,0])
# plt.plot(np.array(x_rec)[:,1])
# plt.subplot(2,2,2)
# plt.plot(np.array(delta_rec)[:,0])
# plt.plot(np.array(delta_rec)[:,1])
# plt.subplot(2,2,3)
# plt.plot(np.array(yaw_rec)[:,0])
# plt.plot(np.array(yaw_rec)[:,1])
# plt.subplot(2,2,4)
# plt.plot(np.array(derdelta_rec)[:,0])
plt.show()
