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
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
from obstacled_environments.common.my_staticSubGoal import StaticSubGoal
import numpy as np
from nonlinear_mpc.acados_settings import *
import nonlinear_mpc.bicycle_model
import nonlinear_mpc.mpc_plot
import nonlinear_mpc.obstacle
from nonlinear_mpc.mpc_controller import mpc_controller
import nonlinear_mpc.path
import time

def run_prius(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "complex-parking-lot-env-v0",
        dt=0.01, robots=robots, render=render
    )

    action = np.array([0., 0.])
    pos0 = np.array([18., -9, 0])
    ob = env.reset(pos=pos0)

    dynamicObst1Dict = {
        "type": "sphere",
        "geometry": {"trajectory": ['-25 + 0.8 * t', '15.', '2.'], "radius": 2.},
    }
    dynamicSphereObst1 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst1Dict)
    env.add_obstacle(dynamicSphereObst1)

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

    print(f"Initial observation : {ob}")
    history = []
    a = np.load("hybrid_astar/ref_largescale_dummy.npy")
    x, y, yaw, direction = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    hybridastar_path = nonlinear_mpc.path.PATH(x, y, yaw, 3)
    speed_profile = nonlinear_mpc.path.get_velprofile(hybridastar_path, 2., 0.2)
    test_param = test_param = {
        "T": 60,
        "N": 12,  # Predict Horizon
        "Tf": 6,  # dim of state space
        "dt": 0.5,  # time step
        "d_dist": 0.4 / 4,  # todo:distance between nodes
        "N_IND": 10,  # search index number
        "lr": 1.425,
        "L": 2.85,  # lr+lf
        "disc_offset": 1,
        "radius": 1.455,
        "start_vel": 0.5,
        "approximate_acc": 0.2,
        "max_acc": 1.0,
        "max_steer_vel": 0.6,
        "xlim ": (-25, 25),
        "ylim ": (-25, 25),
    }
    state = nonlinear_mpc.bicycle_model.ROBOT_STATE(x=18, y=-9, yaw=0, v=0.0)
    T = test_param["T"]
    Tf = test_param["Tf"]  # prediction horizon
    N = test_param["N"]  # number of discretization steps
    disc_offset = test_param["disc_offset"]
    radius = test_param["radius"]
    Nsim = int(T * N / Tf)
    state.get_state(ob)
    obs = []
    obs.append(nonlinear_mpc.obstacle.circle(0, 15, 2))
    mpc = mpc_controller(hybridastar_path, test_param, state, speed_profile,obs)
    t = 0
    for i in range(n_steps):
        # refine obstacle and update obs = []
        obs_x = -25 + 0.8 * t
        obs = [nonlinear_mpc.obstacle.circle(obs_x, 15, 2)]
        # obs_x = 22 - 1 * t
        # obs = [nonlinear_mpc.obstacle.circle(obs_x, 14, 2)]
        mpc.update_obstacles(obs)
        u, ref, x_pred = mpc.control()
        acc = u[0]
        delta_dot = u[1]
        for j in range(int(test_param["dt"] / 0.01)):
            v = state.v + acc * 0.01
            action = [v, delta_dot]
            if abs(ob['robot_0']['joint_state']['steering']) > 0.4:
                if np.sign(action[1]) == np.sign(ob['robot_0']['joint_state']['steering']):
                    action[1] = 0
                    print("modified")
            ob, _, _, _ = env.step(action)
            print(state.delta)
            state.get_state(ob)
            t+=0.01
                # time.sleep(0.01)
        mpc.visualize(ref, x_pred, test_param["xlim "], test_param["ylim "])
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":

    run_prius(render=True)

