from casadi import *

def bicycle_model():
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "kinematicbicycle_model"

    Lr = 1.25
    L =  2.5

    x = MX.sym("x")
    y = MX.sym("y")
    v = MX.sym("v")
    phi = MX.sym("phi")
    delta = MX.sym("delta")
    acc = MX.sym("acc")
    derdelta = MX.sym("derdelta")

    xdot = MX.sym("xdot")
    ydot = MX.sym("ydot")
    phidot = MX.sym("phidot")


    x = vertcat(x, y, v, phi, delta)
    xdot = vertcat(xdot, ydot, acc, phidot, derdelta)
    u = vertcat(acc,derdelta)
    beta = atan(Lr / L * tan(delta))

    f_expl = vertcat(
        v * cos(phi+beta),
        v * sin(phi+beta),
        acc,
        v * sin(beta) / Lr,
        derdelta,
    )

    # state bounds
    model.v_min = -5.0
    model.v_max = 10.0

    model.delta_min = -0.20  # minimum steering angle [rad]
    model.delta_max = 0.20  # maximum steering angle [rad]

    # input bounds
    model.derdelta_min = -1.0  # minimum change rate of stering angle [rad/s]
    model.derdelta_max = 1.0  # maximum change rate of steering angle [rad/s]
    model.acc_min = -1  # -10.0  # minimum throttle change rate
    model.acc_max = 1  # 10.0  # maximum throttle change rate

    # algebraic variables
    z = vertcat([])
    # parameters
    p = vertcat([])

    constraint.expr = vertcat(v, delta)

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
if __name__ == '__main__':
    model = bicycle_model()