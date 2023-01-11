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


def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "complex-parking-lot-env-v0",
        dt=0.01, robots=robots, render=render
    )

    action = np.array([0., 0.])
    pos0 = np.array([-10., 2.6, -np.pi/2])
    ob = env.reset(pos=pos0)

    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-0.5,2.6,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-0.5,-2.6,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-3.0,-2.6,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-7.5,2.6,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-8.0,-2.6,0.0]],place_height=0.)

    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-10.1,-3.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-12.5,2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-16.5,2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-19.0,-2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-21.0,-2.6,0.0]],place_height=0.)

    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-21.0,2.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[4.0,2.6,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[1.0,12.0,1.0],poses_2d=[[5.6,0.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[7.0,7.0,1.0],poses_2d=[[20.5,16.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[8.0,3.0,1.0],poses_2d=[[20.0,1.5,0.0]],place_height=0.)

    env.add_shapes(shape_type='GEOM_BOX',dim=[3.0,18.0,1.0],poses_2d=[[22.5,-9.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[4.0,2.0,1.0],poses_2d=[[19.0,-5.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[4.0,2.0,1.0],poses_2d=[[19.0,-7.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[4.5,2.0,1.0],poses_2d=[[18.75,-11.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[4.5,2.0,1.0],poses_2d=[[18.75,-14.0,0.0]],place_height=0.)
    
    env.add_shapes(shape_type='GEOM_BOX',dim=[4.5,2.0,1.0],poses_2d=[[18.75,-16.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[8.5, 17.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[6.0,18.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[1.0, 17.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-10.0,16.5,0.0]],place_height=0.)
    
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-18.0,16.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-20.5,16.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-22.5,16.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[10.0,2.0,1.0],poses_2d=[[3.0,-12.0,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[6.0,2.0,1.0],poses_2d=[[1.0,-17.0,0.0]],place_height=0.)

    env.add_shapes(shape_type='GEOM_BOX',dim=[1.0,7.0,1.0],poses_2d=[[-8.5,-14.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-10.5,-15.5,0.0]],place_height=0.)
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,5.0,1.0],poses_2d=[[-13.5,-15.5,0.0]],place_height=0.)
    
    env.add_walls()
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

