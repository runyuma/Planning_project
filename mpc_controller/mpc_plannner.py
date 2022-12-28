import cvxpy
import numpy as np
import matplotlib.pyplot as plt
import cvxpy
import car_model
import path
param ={"N":6,   # Predict Horizon
        "Nx":4,  # dim of state space
        "dt":0.2, #time step
        "d_dist":1,#todo:distance between nodes
        "N_IND":10, # search index number
        "lr":1.25,
        "L":2.5,    # lr+lf
        "Q":np.diag([10,10,5,10]),
        "R":np.diag([5,5]),
        "start_vel":1,
        "max_vel":10,
        "min_vel":-5,
        "max_steer":np.deg2rad(20),
        "min_steer":-np.deg2rad(20),
        "max_acc":1,
        "min_acc":-1,
        }
class MPC_PLANNER():
    def __init__(self):
        pass
class ROBOT_STATE():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,delta = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = delta
        #todo:delta
    def state_update(self, acc, delta,param):
        """
        update the state of the robot
        :param acc: acceleration
        :param delta: steering angle
        :param param: model parameters
        """
        L = param["L"]
        dt = param["dt"]
        beta = np.arctan(np.tan(delta)*param["lr"]/L)
        self.x += self.v * np.cos(self.yaw+beta) * dt
        self.y += self.v * np.sin(self.yaw+beta) * dt
        self.yaw += self.v / param["lr"] * np.sin(beta)
        self.v += acc * dt
        self.delta = delta
def get_velprofile(cx, cy, cyaw, target_speed):
    """
        design appropriate speed strategy
        :param cx: x of reference path [m]
        :param cy: y of reference path [m]
        :param cyaw: yaw of reference path [m]
        :param target_speed: target speed [m/s]
        :return: paths,speed profiles
        """

    speed_profile = [target_speed] * len(cx)
    direction = [1.0 for i in range(len(cx))]  # forward

    # Set stop point
    dx = [cx[i + 1] - cx[i] for i in range(len(cx) - 1)]
    dy = [cy[i + 1] - cy[i] for i in range(len(cy) - 1)]
    dyaw = [cyaw[i + 1] - cyaw[i] for i in range(len(cyaw) - 1)]
    # decide the direction of velocity
    for i in range(len(cx) - 1):
        move_direction = np.arctan2(dy[i], dx[i])
        if dx != 0.0 and dy != 0.0:
            dangle = move_direction - cyaw[i]
            if abs(dangle)>=np.pi:
                dangle += -2* np.sign(dangle)*np.pi
            if abs(dangle) >= np.pi / 4.0:
                direction[i] = -1.0
            else:
                direction[i] = 1.0
    for i in range(len(direction)):
        if direction[i] != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
    # if change direction, set vel = 0
    split = []
    for i in range(len(cx)-1):
        if direction[i] != direction[i+1]:
            split.append(i)
            speed_profile[i] = 0.0
            if i != 0:
                for j in range(int(target_speed/0.2)):
                    if direction[i] > 0:
                        speed_profile[i - j] = j*0.2
                        speed_profile[i + j] = - 0.2 * j
                    else:
                        speed_profile[i - j] = -j * 0.2
                        speed_profile[i + j] = 0.2 * j
    for j in range(int(target_speed/0.2)):
        if direction[1] > 0:
            speed_profile[j] = j * 0.2
        else:
            speed_profile[j] = -j * 0.2
    for j in range(int(target_speed / 0.2)):
        if direction[len(cx)-2] > 0:
            speed_profile[len(cx) - j-1] = j * 0.2
        else:
            speed_profile[len(cx) - j-1] = -j * 0.2

    speed_profile[0] = 0.0
    speed_profile[-1] = 0.0

    return speed_profile

def get_reftraj(robot_state,ref_path,vel_profile,param):

    Nx = param["Nx"]
    N = param["N"]
    dt = param["dt"]
    d_dist = param["d_dist"]
    z_ref = np.zeros((Nx, N + 1))
    length = ref_path.length
    start_vel = param["start_vel"]

    ind, err,theta_err = ref_path.nearest_index(robot_state,param)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = vel_profile[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0
    for i in range(1, N + 1):
        dist_move += max(abs(robot_state.v) * dt,start_vel*dt)
        ind_move = int(round(dist_move / d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = vel_profile[index]
        z_ref[3, i] = ref_path.cyaw[index]
    if z_ref[2, 0] != 0:
        direction = np.sign(z_ref[2, 0])
    else:
        indx = min(ind+1, length - 1)
        direction = np.sign(vel_profile[indx])

    return z_ref, ind, direction

def nonlinear_mpc_control(z_ref,initial_state,model,param):
    #todo
    T = param["N"]
    Q = param["Q"]
    R = param["R"]

    x = cvxpy.Variable((2, T + 1))
    v = cvxpy.Variable((1, T + 1))
    phi = cvxpy.Variable((1, T + 1))
    delta = cvxpy.Variable((1, T + 1))
    u = cvxpy.Variable((2, T+1))

    cost = 0.
    constraints = []

    x0 = np.array([initial_state.x,initial_state.y])
    v0 = np.array([initial_state.v])
    yaw0 = np.array([initial_state.yaw])
    delta0 = np.array([initial_state.delta])
    constraints += [x[:, 0] == x0]
    constraints += [v[:, 0] == v0]
    constraints += [phi[:, 0] == yaw0]
    constraints += [delta[:, 0] == delta0]

    for k in range(T):
        #
        x_next,v_next,phi_next,delta_next = model.nonlinear_discrete_model(x[:, k],v[:, k],phi[:, k],delta[:, k],u[:, k],param)
        constraints += [x[:, k + 1] == x_next]
        constraints += [v[:, k + 1] == x_next]
        constraints += [phi[:, k + 1] == x_next]
        constraints += [delta_next[:, k + 1] == x_next]
        cost += cvxpy.quad_form(x[:, k]- z_ref[:2,k], Q[:2,:2])
        cost += cvxpy.quad_form(v[:, k] - z_ref[2:3, k], Q[2:3, 2:3])
        cost += cvxpy.quad_form(phi[:, k] - z_ref[3:4, k], Q[3:4, 3:4])
        if k == 0:
            cost += cvxpy.quad_form(u[:1, k], R[0:1, 0:1])
            cost += cvxpy.quad_form(u[1:2, k]-delta0, R[1:2, 1:2])
        else:
            cost += cvxpy.quad_form(u[:1, k], R[0:1, 0:1])
            cost += cvxpy.quad_form(u[1:2, k] - u[1:2, k-1], R[1:2, 1:2])
    cost += cvxpy.quad_form(x[:, T] - z_ref[:2, T], Q[:2, :2])
    cost += cvxpy.quad_form(v[:, T] - z_ref[2:3, T], Q[2:3, 2:3])
    cost += cvxpy.quad_form(phi[:, T] - z_ref[3:4, T], Q[3:4, 3:4])
    #solve
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.OSQP)
    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = x.value
        v = v.value
        phi = phi.value
        delta = delta.value
        u = u.value
    else:
        print("Cannot solve nonlinear mpc!")
    return x,v,phi,delta,u

def linear_mpc_control(z_ref, initial_state,direction, model, param):
    # todo
    start_vel = param["start_vel"]
    T = param["N"]
    Q = param["Q"]
    R = param["R"]

    x = cvxpy.Variable((4, T + 1))
    u = cvxpy.Variable((2, T + 1))

    cost = 0.
    constraints = []

    x0 = np.array([initial_state.x, initial_state.y,initial_state.v,initial_state.yaw])
    delta0 = np.array([initial_state.delta])
    constraints += [x[:, 0] == x0]

    # if abs(initial_state.v) == 0 and z_ref[2][0] == 0:
    if abs(initial_state.v) <= 0.1 and z_ref[2][0] == 0:
        v0 = start_vel * direction
        print("starting acc")
    else:
        v0 = initial_state.v
    A, B, C = model.linear_discrete_model(v0, initial_state.yaw, initial_state.delta, param)
    sbeta0 = car_model.delta2sbeta(initial_state.delta,param)
    # in case of angle overflow
    for i in range(T + 1):
        if abs(initial_state.yaw-z_ref[3][i])>=np.pi:
            z_ref[3][i] = z_ref[3][i] + 2*np.pi*np.sign(initial_state.yaw-z_ref[3][i])

    for k in range(T):
        constraints += [x[:, k + 1] == A@x[:, k]+B@u[:, k]+C]
        cost += cvxpy.quad_form(x[:, k] - z_ref[:, k], Q)
        if k == 0:
            cost += cvxpy.quad_form(u[:1, k], R[0:1, 0:1])
            cost += cvxpy.quad_form(u[1:2, k] - np.array([sbeta0]), R[1:2, 1:2])
        else:
            cost += cvxpy.quad_form(u[:1, k], R[0:1, 0:1])
            cost += cvxpy.quad_form(u[1:2, k] - u[1:2, k - 1], R[1:2, 1:2])
        constraints += [u[0, k] <= param["max_acc"]]
        constraints += [u[0, k] >= param["min_acc"]]
        constraints += [u[1, k] <= car_model.delta2sbeta(param["max_steer"],param)]
        constraints += [u[1, k] >= car_model.delta2sbeta(param["min_steer"],param)]
        constraints += [u[1, k + 1] - u[1, k] <= param["dt"]*param["max_dsteer"]]
        constraints += [u[1, k+1] - u[1, k] >= param["dt"]*param["min_dsteer"]]
    #todo add u0 dsteer constraint

    cost += cvxpy.quad_form(x[:, T] - z_ref[:, T], Q)
    constraints += [u[1, 0]-sbeta0 <= param["dt"]*param["max_dsteer"]]
    constraints += [u[1, 0]-sbeta0 >= param["dt"]*param["min_dsteer"]]
    constraints += [u[0, T] <= param["max_acc"]]
    constraints += [u[0, T] >= param["min_acc"]]
    constraints += [u[1, T] <= car_model.delta2sbeta(param["max_steer"],param)]
    constraints += [u[1, T] >= car_model.delta2sbeta(param["min_steer"],param)]
    # solve
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.OSQP)
    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = x.value
        u = u.value
    else:
        print("Cannot solve linear mpc!")

    return x,u


