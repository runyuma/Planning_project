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

    env.add_shapes(shape_type='GEOM_BOX',dim=[12.0,1.0,2.0],poses_2d=[[2.0,-2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[7.5,3.0,2.0],poses_2d=[[-0.25,3.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[6.0,3.0,2.0],poses_2d=[[13.0,3.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[20.0,1.0,2.0],poses_2d=[[6.0,5.0,0.0]],place_height=0.)

    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        # TODO: use the real-time MPC (with 'sensor feedback' of the simulation environment)
        #       to predict the differential actions for the next step.
        action = getActionUpdate(action,i,diff_action)
        ob, _, _, _ = env.step(action)
        # TODO: check definition of ob, and output [x,y,yaw,d_delta],
        #       and transfer it to the MPC controller, instead of using the 
        #       model-predicted state
        # obs_states.append([]) 
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":


    run_prius(render=True)

