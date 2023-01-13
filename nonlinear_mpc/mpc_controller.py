import matplotlib.pyplot as plt
import numpy as np
import time, os
import nonlinear_mpc.path as path
from nonlinear_mpc.acados_settings import *
import nonlinear_mpc.bicycle_model as bicycle_model
import nonlinear_mpc.mpc_plot as mpc_plot
import nonlinear_mpc.obstacle as obstacle

class mpc_controller():
    def __init__(self,astarpath,param,init_state,speed_profile,obstacles):
        self.astarpath = astarpath
        self.param = param
        self.state = init_state
        init_state_array = np.array([init_state.x, init_state.y, init_state.v, init_state.yaw, init_state.delta])
        obstacle_num = len(obstacles)
        self.obstacles = obstacles
        self.constraint, self.model,self.acados_solver = acados_settings(self.param["Tf"], self.param["N"], init_state_array, obstacle_num)
        self.speed_profile = speed_profile
    def control(self):
        plt.cla()
        N = self.param["N"]
        ref, ind, dir = path.get_reftraj(self.state, self.astarpath, self.speed_profile, self.param)
        x0 = np.array([self.state.x, self.state.y, self.state.v, self.state.yaw, self.state.delta])
        # print("x0",x0)
        self.acados_solver.set(0, "lbx", x0)
        self.acados_solver.set(0, "ubx", x0)
        self.frame_constraint(ref)
        for j in range(N):
            yref = np.zeros(7)
            yref[:5] = np.array(ref[:, j])
            self.acados_solver.set(j, "yref", yref)
        # solve
        self.acados_solver.set(N, "yref", np.array(ref[:, N]))
        status = self.acados_solver.solve()
        if status != 0:
            print("wrong")
            # print("acados returned status {} in closed loop iteration {}.".format(status, i))

        x_pred = [self.acados_solver.get(j, "x") for j in range(N + 1)]
        u = [self.acados_solver.get(j, "u") for j in range(N)]

        return u[0],ref,x_pred
    def update_obstacles(self,obstacles):
        self.obstacles = obstacles
    def frame_constraint(self,ref):
        disc_offset = self.param["disc_offset"]
        N = self.param["N"]
        for j in range(N + 1):
            hyperplanes = []
            for obs in self.obstacles:
                obs.plot()
                h = obs.find_hyperplane(self.state, ref[:, j], 1.25)
                if h is not None:
                    obs.draw_hyperplane(h)
                hyperplanes.append(h)
                # print("h:", h)
            constraint_num = len(hyperplanes)
            nx = self.model.x.size()[0]
            nu = self.model.u.size()[0]
            C = np.zeros((constraint_num * 2, nx))
            D = np.zeros((constraint_num * 2, nu))
            g_l = -1000 * np.ones(constraint_num * 2)
            g_u = 1000 * np.ones(constraint_num * 2)
            for i in range(constraint_num):
                if hyperplanes[i] is not None:
                    h = hyperplanes[i]
                    T = np.array([[1, 0, 0, -np.sin(self.state.yaw) * disc_offset, 0],
                                  [0, 1, 0, np.cos(self.state.yaw) * disc_offset, 0],
                                  [1, 0, 0, np.sin(self.state.yaw) * disc_offset, 0],
                                  [0, 1, 0, -np.cos(self.state.yaw) * disc_offset, 0]])
                    t = np.array([disc_offset * np.cos(self.state.yaw) + self.state.yaw * np.sin(self.state.yaw) * disc_offset,
                                  disc_offset * np.sin(self.state.yaw) - self.state.yaw * np.cos(self.state.yaw) * disc_offset,
                                  -disc_offset * np.cos(self.state.yaw) - self.state.yaw * np.sin(self.state.yaw) * disc_offset,
                                  -disc_offset * np.sin(self.state.yaw) + self.state.yaw * np.cos(self.state.yaw) * disc_offset])
                    L = np.array([[h[0], h[1], 0, 0],
                                  [0, 0, h[0], h[1]]])
                    C[i * 2:(i + 1) * 2, :] = L @ T
                    g_u[i * 2:(i + 1) * 2] = np.array([-h[2], -h[2]]) - L @ t
                    g_l[i * 2:(i + 1) * 2] = np.array([-np.inf, -np.inf])

            self.acados_solver.constraints_set(j, "C", C, api='new')
            if j < N:
                self.acados_solver.constraints_set(j, "D", D, api='new')
            self.acados_solver.constraints_set(j, "lg", g_l)
            self.acados_solver.constraints_set(j, "ug", g_u)
    def visualize(self,ref,x_pred,xlim,ylim):
        mpc_plot.plot_mpc(self.astarpath.cx, self.astarpath.cy, self.astarpath.cyaw, ref, x_pred,
                          self.speed_profile)
        mpc_plot.draw_car(self.state.x, self.state.y, self.state.yaw, self.state.delta)
        plt.xlim(xlim[0], xlim[1])
        plt.ylim(ylim[0], ylim[1])
        plt.title("Linear MPC, " + "v = " + str(self.state.v) + "\n delta = " + str(self.state.delta))
        plt.pause(0.01)