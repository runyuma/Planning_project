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
from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal

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

# yongxi
# ----------------------------------------------------------------------------------------------------
    x_pred = np.array([[1.,1.,0.1],[2.,2.,0.1],[3.,3.,0.1],[4.,4.,0.1]])
    MPC_PRED_LEN = 4
    mpc_preds_list = []
    goals = []

    for i in range(MPC_PRED_LEN):
        goal1Dict = {
            "weight": 1.0, "is_primary_goal": True, 'indices': [0, 1, 2], 'parent_link': 0, 'child_link': 3,
            'desired_position': [0., 0, 0.], 'epsilon': 0.2, 'type': "staticSubGoal", 
        }
        mpc_preds_list.append(goal1Dict)

        goal = StaticSubGoal(name="goal"+str(i), content_dict=goal1Dict)
        goals.append(goal)
    for i in range(MPC_PRED_LEN):
        env.add_goal(goals[i])
    # test_pos_dict = {'step1':np.array([1.,1.]),'step2':np.array([2.,2.]),'step2':np.array([3.,3.])}

    # mpcGoalDict = {
    #     "weight": 1.0, "is_primary_goal": True, 'indices': [0, 1, 2], 'parent_link': 0, 'child_link': 3,
    #     'trajectory': test_pos_dict, 'epsilon': 0.08, 'type': "selfDefinedSubgoal", 
    # }

    # our_mpc_goal = DynamicSubGoal(name='mpc_goal',content_dict=mpcGoalDict)
    # env.add_goal(our_mpc_goal)
# ----------------------------------------------------------------------------------------------------
    env.add_shapes(shape_type='GEOM_BOX',dim=[2.0,2.0,2.0],poses_2d=[[3.0,3.0,0.0]],place_height=0.)

    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
# yongxi
# ----------------------------------------------------------------------------------------------------
        # dummy update new predictions
        # update the goal dictionary used for
        for i in range(MPC_PRED_LEN):
            x_pred[i][0] += 0.1
            goals[i].update_position(x_pred[i].tolist())
            # env.add_goal(goals[i])
# ----------------------------------------------------------------------------------------------------

        ob, _, _, _ = env.step(action)
        # if ob['robot_0']['joint_state']['steering'] > 0.2:
        #     action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":

    run_prius(render=True)

