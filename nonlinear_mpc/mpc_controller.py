import matplotlib.pyplot as plt
import numpy as np
import time, os
import nonlinear_mpc.path as path
from nonlinear_mpc.acados_settings import *
import nonlinear_mpc.bicycle_model as bicycle_model
import nonlinear_mpc.mpc_plot as mpc_plot
import nonlinear_mpc.obstacle as obstacle

class mpc_controller():
    def __init__(self,astarpath,param,init_state,speed_profile):
        self.astarpath = astarpath
        self.param = param
        self.state = init_state
        init_state_array = np.array([init_state.x, init_state.y, init_state.v, init_state.yaw, init_state.delta])
        self.constraint, self.model,self.acados_solver = acados_settings(self.param["Tf"], self.param["N"], init_state_array)
        self.speed_profile = speed_profile
    def control(self):
        N = self.param["N"]
        ref, ind, dir = path.get_reftraj(self.state, self.astarpath, self.speed_profile, self.param)
        x0 = np.array([self.state.x, self.state.y, self.state.v, self.state.yaw, self.state.delta])
        # print("x0",x0)
        self.acados_solver.set(0, "lbx", x0)
        self.acados_solver.set(0, "ubx", x0)

        for j in range(N):
            yref = np.zeros(7)
            yref[:5] = np.array(ref[:, j])
            self.acados_solver.set(j, "yref", yref)
        # solve
        self.acados_solver.set(N, "yref", np.array(ref[:, N]))
        status = self.acados_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration {}.".format(status, i))
        x_pred = [self.acados_solver.get(j, "x") for j in range(N + 1)]
        u = [self.acados_solver.get(j, "u") for j in range(N)]

        return u[0],ref,x_pred
    def visualize(self,ref,x_pred):
        plt.cla()
        mpc_plot.plot_mpc(self.astarpath.cx, self.astarpath.cy, self.astarpath.cyaw, ref, x_pred,
                          self.speed_profile)
        mpc_plot.draw_car(self.state.x, self.state.y, self.state.yaw, self.state.delta)
        plt.xlim(0, 12)
        plt.ylim(-6, 6)
        plt.title("Linear MPC, " + "v = " + str(self.state.v) + "\n delta = " + str(self.state.delta))
        plt.pause(0.01)