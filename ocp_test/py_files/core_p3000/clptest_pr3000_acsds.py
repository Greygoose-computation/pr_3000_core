import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


def make_p3000_plant_casadi():
    theta_f = ca.SX.sym("theta_f")
    omega_f = ca.SX.sym("omega_f")
    v_trl = ca.SX.sym("v_trl")
    omega_trl = ca.SX.sym("omega_trl")
    theta_trl = ca.SX.sym("theta_trl")
    x = ca.vertcat(theta_f, omega_f, v_trl, omega_trl, theta_trl)

    tau1 = ca.SX.sym("tau1")
    tau2 = ca.SX.sym("tau2")
    u = ca.vertcat(tau1, tau2)

    r_dw = ca.SX.sym("r_dw")
    r_dwl = ca.SX.sym("r_dwl")
    l_wb = ca.SX.sym("l_wb")
    r_trl = ca.SX.sym("r_trl")
    f_ric = ca.SX.sym("f_ric")
    m_robot = ca.SX.sym("m_robot")
    m_payload = ca.SX.sym("m_payload")
    g = ca.SX.sym("g")
    I_zz = ca.SX.sym("I_zz")
    l_lcom = ca.SX.sym("l_lcom")
    I_pivot = ca.SX.sym("I_pivot")

    p = ca.vertcat(r_dw, r_dwl, l_wb, r_trl, f_ric, m_robot, m_payload, g, I_zz, l_lcom, I_pivot)

    M = m_robot + m_payload
    I = I_zz + m_payload * (l_lcom ** 2)

    eps = 1e-3
    sig_v = v_trl / ca.sqrt(v_trl**2 + eps)
    sig_w = omega_trl / ca.sqrt(omega_trl**2 + eps)

    F_rect = ((tau1 + tau2) / r_dwl) * ca.cos(theta_f)
    F_rot  = ((tau1 - tau2) / r_dwl) * ca.cos(theta_f)

    accel_trl = (F_rect - f_ric * M * g * sig_v) / M
    alpha_trl = (F_rot * l_wb - f_ric * M * g * r_trl * sig_w) / I

    theta = theta_f
    accel_f = accel_trl * ca.cos(theta) + (alpha_trl * l_wb) * ca.sin(theta)

    dd_theta_f = (
        ((tau1 - tau2) / r_dwl) * (2 * r_dw)
        - accel_f * ca.sin(theta_f) / l_wb
    ) / I_pivot

    xdot = ca.vertcat(
        omega_f,
        dd_theta_f,
        accel_trl,
        alpha_trl,
        omega_trl
    )

    return ca.Function("f_p3000", [x, u, p], [xdot], ["x", "u", "p"], ["xdot"])


def build_acados_model():
    f = make_p3000_plant_casadi()

    x = ca.SX.sym("x", 5)
    u = ca.SX.sym("u", 2)
    p = ca.SX.sym("p", 11)

    xdot = f(x=x, u=u, p=p)["xdot"]

    model = AcadosModel()
    model.name = "p3000_mpc_model"
    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = xdot
    return model


def build_ocp(N, dt, u_min, u_max, Q, R, QN):
    nx, nu = 5, 2
    ny = nx + nu

    ocp = AcadosOcp()
    ocp.model = build_acados_model()

    ocp.solver_options.tf = N * dt
    ocp.solver_options.N_horizon = N  # recommended

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    W = np.block([[Q, np.zeros((nx, nu))],
                  [np.zeros((nu, nx)), R]])
    ocp.cost.W = W
    ocp.cost.W_e = QN

    Vx = np.zeros((ny, nx))
    Vu = np.zeros((ny, nu))
    Vx[:nx, :] = np.eye(nx)
    Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0, 1], dtype=int)

    ocp.constraints.idxbx = np.arange(nx, dtype=int)
    ocp.constraints.lbx = np.zeros(nx)
    ocp.constraints.ubx = np.zeros(nx)

    ocp.dims.np = 11
    ocp.parameter_values = np.zeros(11)

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 50

    # where generated C code will go:
    ocp.code_export_directory = "c_generated_code_p3000"

    return ocp


def main():
    dt = 0.1
    N = 50

    tau_max = 5.0
    u_min = np.array([-tau_max, -tau_max], dtype=float)
    u_max = np.array([ tau_max,  tau_max], dtype=float)

    Q = np.diag([5.0, 5.0, 5.0, 5.0, 5.0])
    R = np.diag([0.05, 0.05])
    QN = np.diag([15.0*5.0, 15.0*5.0, 60.0*5.0, 15.0*5.0, 80.0*5.0])

    ocp = build_ocp(N, dt, u_min, u_max, Q, R, QN)

    # This triggers code generation + compilation artifacts for the solver
    solver = AcadosOcpSolver(ocp, json_file="p3000_acados_ocp.json")

    print("Generated C code in:", ocp.code_export_directory)
    print("JSON file:", "p3000_acados_ocp.json")


if __name__ == "__main__":
    main()
