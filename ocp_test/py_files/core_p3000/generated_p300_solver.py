import numpy as np
import casadi as ca

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


def p3000_dynamics_casadi():
    # x = [theta_f, omega_f, v_trl, omega_trl, psi, px, py]
    x = ca.SX.sym("x", 7)
    u = ca.SX.sym("u", 2)

    theta_f, omega_f, v_trl, omega_trl, psi, px, py = x[0], x[1], x[2], x[3], x[4], x[5], x[6]
    tau1, tau2 = u[0], u[1]

    # parameters as CasADi symbols (packed into p)
    p = ca.SX.sym("p", 11)
    r_dw, r_dwl, l_wb, f_ric, m_robot, m_payload, g, I_zz, m_pay, l_lcom, I_pivot = \
        p[0],  p[1],  p[2],  p[3],  p[4],    p[5],      p[6], p[7], p[8], p[9],  p[10]

    M = m_robot + m_payload
    I = I_zz + m_pay*(l_lcom**2)

    eps = 1e-3
    vel_sig = v_trl / ca.sqrt(v_trl**2 + eps)

    # assumption: theta = theta_f
    theta = theta_f
    Vf = (omega_trl*l_wb)*ca.sin(theta) + (v_trl*ca.cos(theta))

    F_rect = (tau1 + tau2) * r_dwl * ca.cos(theta_f)
    F_rot  = (tau1 - tau2) * r_dwl * ca.cos(theta_f)

    dd_theta_f = (2*(r_dw**2)*(tau1 - tau2) - (Vf*ca.cos(theta_f)/l_wb)) / I_pivot
    alpha_trl  = (F_rot*l_wb - f_ric*M*g*vel_sig) / I
    accel_trl  = (F_rect     - f_ric*M*g*vel_sig) / M

    xdot = ca.vertcat(
        omega_f,
        dd_theta_f,
        accel_trl,
        alpha_trl,
        omega_trl,
        v_trl*ca.cos(psi),
        v_trl*ca.sin(psi),
    )
    return x, u, p, xdot


def main():
    ocp = AcadosOcp()

    # --- model ---
    model = AcadosModel()
    model.name = "p3000"

    x, u, p, f_expl = p3000_dynamics_casadi()
    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    ocp.model = model

    nx = 7
    nu = 2

    # --- horizon ---
    N = 15
    Tf = 1.5
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # --- cost (minimal quadratic tracking) ---
    # Use linear least squares cost on y = [x; u]
    ny = nx + nu
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    Vx = np.zeros((ny, nx))
    Vu = np.zeros((ny, nu))
    Vx[:nx, :nx] = np.eye(nx)
    Vu[nx:,  :] = np.eye(nu)
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    Q = np.diag([1.0, 0.1, 2.0, 0.1, 0.2, 10.0, 10.0])
    R = np.diag([0.01, 0.01])
    W = np.block([
        [Q,               np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R],
    ])
    ocp.cost.W = W
    ocp.cost.W_e = 5.0 * Q

    # references (set at runtime in C++ each solve)
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((nx,))

    # --- constraints ---
    tau_max = 5.0
    ocp.constraints.lbu = np.array([-tau_max, -tau_max])
    ocp.constraints.ubu = np.array([ tau_max,  tau_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # initial state constraint x0 (set at runtime)
    ocp.constraints.x0 = np.zeros((nx,))

    # --- solver options (fast NMPC defaults) ---
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1

    # code export directory
    ocp.code_export_directory = "c_generated_code"

    # build solver code + compile shared lib (depends on your acados setup)
    solver = AcadosOcpSolver(ocp, json_file="p3000_ocp.json")
    print("Generated solver in:", ocp.code_export_directory)


if __name__ == "__main__":
    main()
