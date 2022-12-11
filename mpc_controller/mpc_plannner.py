import cvxpy
import numpy as np
import matplotlib.pyplot as plt
mpc_param ={"N":6,   # Predict Horizon
            "Nx":4,  # dim of state space
            "dt":0.2,
            }
class MPC_PLANNER():
    def __init__(self):
        pass
def get_velprofile():
    pass
def get_reftraj(robot_state,ref_path,vel_profile):

    Nx = mpc_param["Nx"]
    N = mpc_param["N"]
    dt = mpc_param["dt"]
    d_dist = mpc_param["d_dist"]
    z_ref = np.zeros((Nx, N + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(robot_state)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = vel_profile[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0

    for i in range(1, N + 1):
        dist_move += abs(robot_state.v) * dt
        ind_move = int(round(dist_move / d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = vel_profile[index]
        z_ref[3, i] = ref_path.cyaw[index]

    return z_ref, ind
