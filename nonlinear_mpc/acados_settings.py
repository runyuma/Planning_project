from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from nonlinear_mpc.bicycle_model import bicycle_model
import scipy.linalg
import numpy as np
import os
# setting for acados
# docs: https://acados.github.io/acados/python_interface.html
# docs: https://acados.github.io/acados/ocp_nlp.html
def acados_settings(Tf, N,initial_state):
    """
    :param Tf: horizon length
    :param N: number of control intervals
    """
    ocp = AcadosOcp()
    ocp.dims.N = N

    # export model
    model, constraint = bicycle_model(initial_state)

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    model_ac.con_h_expr = constraint.expr

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx
    print("nx: ", nx)
    print("nu: ", nu)
    #
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.diag([10,10,5,20,5,1,100])# todo
    ocp.cost.W_e = np.diag([10,10,5,20,0])# todo

    # cost = ||VxX+VuU-yref||w
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0])

    # set constraints
    ocp.constraints.x0 = model.x0
    # ocp.constraints.lbx = np.array([-12])
    # ocp.constraints.ubx = np.array([12])
    # ocp.constraints.idxbx = np.array([1])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([model.acc_min, model.derdelta_min])
    ocp.constraints.ubu = np.array([model.acc_max, model.derdelta_max])
    ocp.constraints.C = constraint.C
    ocp.constraints.D = constraint.D
    ocp.constraints.lg = constraint.lg
    ocp.constraints.ug = constraint.ug
    ocp.constraints.C_e = constraint.C
    ocp.constraints.lg_e = constraint.lg
    ocp.constraints.ug_e = constraint.ug

    nsh = constraint.expr.shape[0]
    ocp.constraints.lh = np.array(
        [
            -1000,
            -1000,
            -1000,
            -1000,
            model.v_min,
            model.delta_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            1000,
            1000,
            1000,
            1000,
            model.v_max,
            model.delta_max,
        ]
    )
    # ocp.constraints.lsh = np.zeros(nsh)
    # ocp.constraints.ush = np.zeros(nsh)
    # ocp.constraints.idxsh = np.array(range(nsh))

    # setting path
    acados_source_path = "/home/marunyu/study/planning/acados"
    ocp.acados_include_path = acados_source_path + '/include'
    ocp.acados_lib_path = acados_source_path + '/lib'

    #solver
    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    # ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    # ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.qp_solver_tol_eq = 1e-4
    ocp.solver_options.qp_solver_tol_ineq = 1e-4
    ocp.solver_options.qp_solver_tol_comp = 1e-4
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return constraint, model, acados_solver

if __name__ == "__main__":
    acados_settings(1, 10)

