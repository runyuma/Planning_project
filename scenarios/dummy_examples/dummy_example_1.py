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


def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "simple-parking-lot-env-v0",
        dt=0.01, robots=robots, render=render
    )

    action = np.array([1.0, 0.])
    pos0 = np.array([1., 1.,0.])
    ob = env.reset(pos=pos0)

    # obstacles
    # env.add_walls()
    # from obstacled_environments.simple_parking_lot.scene_objects.obstacles import (
    #     sphereObst1,
    #     sphereObst2,
    #     urdfObst1,
    #     dynamicSphereObst3,
    # )

    # env.add_obstacle(sphereObst1)
    # env.add_obstacle(sphereObst2)
    # env.add_obstacle(urdfObst1)
    # env.add_obstacle(dynamicSphereObst3)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,2.0,2.0],poses_2d=[[3.0,3.0,0.0]],place_height=0.)

    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        # if ob['robot_0']['joint_state']['steering'] > 0.2:
        #     action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":

    run_prius(render=True)

