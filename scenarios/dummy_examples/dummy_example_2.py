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

    action = np.array([0., 0.])
    pos0 = np.array([0., 0., 0.])
    ob = env.reset(pos=pos0)



    env.add_shapes(shape_type='GEOM_BOX',dim=[5.5,2.0,2.0],poses_2d=[[-1.25,2.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.5,0.5,2.0],poses_2d=[[2.75,2.75,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[1.0,2.0,2.0],poses_2d=[[4.5,2.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[9.0,1.0,2.0],poses_2d=[[0.5,-1.5,0.0]],place_height=0.)

    # from obstacled_environments.simple_parking_lot.scene_objects.goal import splineGoal
    # env.add_goal(splineGoal)

    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":

    run_prius(render=True)

