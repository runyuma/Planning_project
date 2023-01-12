import numpy as np
from casadi import *

def bicycle_model(initial_state):
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "kinematicbicycle_model"

    Lr = 1.25
    L =  2.5
    disc_offset = 0.5
    radius = 1.25
    constraint_num = 3

    x_ = MX.sym("x_")
    y_ = MX.sym("y_")
    v = MX.sym("v")
    theta = MX.sym("theta")
    delta = MX.sym("delta")
    acc = MX.sym("acc")
    derdelta = MX.sym("derdelta")

    dx = MX.sym("dx")
    dy = MX.sym("dy")
    phidot = MX.sym("phidot")


    x = vertcat(x_, y_, v, theta, delta)
    xdot = vertcat(dx, dy, acc, phidot, derdelta)
    u = vertcat(acc,derdelta)
    beta = atan(Lr / L * tan(delta))

    f_expl = vertcat(
        v * cos(theta+beta),
        v * sin(theta+beta),
        acc,
        v * sin(beta) / Lr,
        derdelta,
    )

    # state bounds
    model.v_min = -5.0
    model.v_max = 10.0

    model.delta_min = -0.40  # minimum steering angle [rad]
    model.delta_max = 0.40  # maximum steering angle [rad]

    # input bounds
    model.derdelta_min = -0.6  # minimum change rate of stering angle [rad/s]
    model.derdelta_max = 0.6  # maximum change rate of steering angle [rad/s]
    model.acc_min = -1  # -10.0  # minimum throttle change rate
    model.acc_max = 1  # 10.0  # maximum throttle change rate

    model.x0 = initial_state

    # algebraic variables
    z = vertcat([])
    # parameters
    p = vertcat([])

    constraint.expr = vertcat(
        x_+cos(theta)*disc_offset,
        y_+sin(theta)*disc_offset,
        x_-cos(theta)*disc_offset,
        y_-sin(theta)*disc_offset,
        v,
        delta)
    constraint.C = np.zeros((constraint_num*2, 5))
    constraint.D = np.zeros((constraint_num * 2, 2))
    constraint.lg = np.array([-1000]*constraint_num*2)
    constraint.ug = np.array([1000]*constraint_num*2)
    model.x = x
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.xdot = xdot
    model.u = u
    model.name = model_name
    params = types.SimpleNamespace()
    params.Lr = Lr
    params.L = L
    model.params = params
    model.z = z
    model.p = p

    return model, constraint
class ROBOT_STATE():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,delta = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = delta
        #todo:delta
    def get_state(self,dict):
        self.x = dict["robot_0"]["joint_state"]["position"][0]
        self.y = dict["robot_0"]["joint_state"]["position"][1]
        self.yaw = dict["robot_0"]["joint_state"]["position"][2]
        self.v = dict["robot_0"]["joint_state"]["forward_velocity"][0]
        self.delta = dict["robot_0"]["joint_state"]["steering"][0]

    def state_update(self, acc, derdelta,param):
        """
        update the state of the robot
        :param acc: acceleration
        :param delta: steering angle
        :param param: model parameters
        """
        L = param["L"]
        dt = param["dt"]
        beta = np.arctan(np.tan(self.delta)*param["lr"]/L)
        self.x += self.v * np.cos(self.yaw+beta) * dt
        self.y += self.v * np.sin(self.yaw+beta) * dt
        self.yaw += self.v / param["lr"] * np.sin(beta)
        self.v += acc * dt
        self.delta += derdelta * dt
if __name__ == '__main__':
    model = bicycle_model()