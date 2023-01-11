import gym
import sys
import os #导入os模块
cur_path = os.getcwd() #得到当前工作目录
sys.path.append(cur_path)
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Planning_project/")
# sys.path.append(cur_path+"/obstacled_environments/common/simple_parking_lot")
# print(sys.path)
from obstacled_environments.simple_parking_lot import urdf_simple_env
from obstacled_environments.common.prius import Prius
from obstacled_environments.common.generic_urdf import GenericUrdfReacher
import numpy as np


def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "simple-parking-lot-env-v0",
        dt=0.01, robots=robots, render=render
    )

    action = np.array([0., 0.])
    pos0 = np.array([0., 0., 0.])
    ob = env.reset(pos=pos0)

    env.add_shapes(shape_type='GEOM_BOX',dim=[12.0,1.0,2.0],poses_2d=[[2.0,-2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[7.5,3.0,2.0],poses_2d=[[-0.25,3.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[6.0,3.0,2.0],poses_2d=[[13.0,3.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[20.0,1.0,2.0],poses_2d=[[6.0,5.0,0.0]],place_height=0.)

    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        action[0] = 0.1
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":

    run_prius(render=True)

