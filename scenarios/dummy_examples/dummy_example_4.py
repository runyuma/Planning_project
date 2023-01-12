import gym
import sys
import os #导入os模块
cur_path = os.getcwd() #得到当前工作目录
sys.path.append(cur_path)
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Planning_project/")
# sys.path.append(cur_path+"/obstacled_environments/common/simple_parking_lot")
# print(sys.path)
from obstacled_environments.complex_parking_lot import urdf_complex_env
from obstacled_environments.common.prius import Prius
from obstacled_environments.common.generic_urdf import GenericUrdfReacher
import numpy as np
import pandas as pd
from nonlinear_mpc.acados_settings import *
import nonlinear_mpc.bicycle_model
import nonlinear_mpc.mpc_plot
import nonlinear_mpc.obstacle
from nonlinear_mpc.mpc_controller import mpc_controller
import nonlinear_mpc.path
import time


def getDiffActions(npy_filename):

    diff_action = []
    dT = 0.4 # timestep of MPC control input
    data_tmp = np.load(npy_filename)
    acc = data_tmp[:,0]
    d_delta = data_tmp[:,1]
    # print(acc.shape)
    sim_time = np.zeros(acc.shape[0])
    for i in range(acc.shape[0]):
        sim_time[i] = dT * i
        sim_time[i] = round(sim_time[i],2)

    acc = acc.transpose().tolist()
    d_delta = d_delta.transpose().tolist()
    sim_time = sim_time.transpose().tolist()
    # print(sim_time)

    collected = {'acc':acc,'d_delta':d_delta,'sim_time':sim_time}
    # print(collected)
    diff_action = pd.DataFrame(collected,dtype=float,index=sim_time)

    return diff_action

def getActionUpdate(action, k, diff_action):

    dT_gym = 0.01
    current_time = k*dT_gym

    idx = round(0.4*float(int(current_time/0.4)),2) # the current time (resolution is 0.4s), is used as index to find proper acc and d_delta
    # print(idx.dtype)
    acc = diff_action.loc[idx,'acc']
    d_delta = diff_action.loc[idx,'d_delta']

    action[0] = action[0] + dT_gym*acc
    action[1] = action[1] + dT_gym*d_delta

    return action

def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):

    f_name = os.path.join(os.path.dirname(__file__), 'seq_small1.npy')
    diff_action = getDiffActions(f_name) # every line of the 'seq.npy' is [longitudinal_accl, d_delta]

    # print('----------test----------')
    # print(diff_action.loc[0.4,'acc'])
    # print('----------test----------')
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "complex-parking-lot-env-v0",
        dt=0.01, robots=robots, render=render
    )

    action = np.array([0., 0.])
    pos0 = np.array([0., 0., 0.])
    ob = env.reset(pos=pos0)
    obs_states = []

    env.add_shapes(shape_type='GEOM_BOX', dim=[12.0, 1.0, 2.0], poses_2d=[[2.0, -2.5, 0.0]], place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX', dim=[7.5, 3.0, 2.0], poses_2d=[[-0.25, 3.0, 0.0]], place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX', dim=[6.0, 3.0, 2.0], poses_2d=[[13.0, 3.0, 0.0]], place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX', dim=[20.0, 1.0, 2.0], poses_2d=[[6.0, 5.0, 0.0]], place_height=0.)

    print(f"Initial observation : {ob}")
    history = []
    a = np.load("ref_simple1.npy")
    x, y, yaw, direction = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    hybridastar_path = nonlinear_mpc.path.PATH(x, y, yaw, 0)
    speed_profile = nonlinear_mpc.path.get_velprofile(hybridastar_path, 1., 0.1)
    test_param = test_param = {
        "T": 60,
        "N": 6,  # Predict Horizon
        "Tf": 2.4,  # dim of state space
        "dt": 0.4,  # time step
        "d_dist": 0.04,  # todo:distance between nodes
        "N_IND": 10,  # search index number
        "lr": 1.425,
        "L": 2.85,  # lr+lf
        "disc_offset": 1,
        "radius": 1.455,
        "start_vel": 0.25,
        "approximate_acc": 0.2,
        "max_acc": 1.0,
        "max_steer_vel": 0.6,
    }
    T = test_param["T"]
    Tf = test_param["Tf"]  # prediction horizon
    N = test_param["N"]
    Nsim = int(T * N / Tf)
    state = nonlinear_mpc.bicycle_model.ROBOT_STATE(x=1.0125, y=0, yaw=0, v=0.0)
    state.get_state(ob)
    mpc = mpc_controller(hybridastar_path, test_param, state, speed_profile)
    for i in range(Nsim):
        u, ref, x_pred = mpc.control()
        acc = u[0]
        delta_dot = u[1]
        for j in range(int(test_param["dt"]/0.01)):
            v = state.v + acc * 0.01
            action = [v,delta_dot]
            ob, _, _, _ = env.step(action)
            state.get_state(ob)
            # time.sleep(0.01)
        mpc.visualize(ref, x_pred)
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)

