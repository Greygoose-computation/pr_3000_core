"""
MPC (CasADi + IPOPT) for the P3000 robot+trailer plant model.

- Plant: xdot = f(x,u,p)
- Discretization: ZOH + RK4 => x_{k+1} = F(x_k, u_k, p)
- MPC: multiple shooting with states X and controls U
- Cost: quadratic tracking to x_ref + control effort
- Constraints: input bounds on tau1, tau2
"""

import casadi as ca
import numpy as np


# -----------------------------
# 1) Plant model: xdot = f(x,u,p)
# -----------------------------
def make_p3000_plant():
    # states: [theta_f, omega_f, v_trl, omega_trl, psi, px, py]
    theta_f   = ca.SX.sym("theta_f")
    omega_f   = ca.SX.sym("omega_f")
    v_trl     = ca.SX.sym("v_trl")
    omega_trl = ca.SX.sym("omega_trl")
    psi       = ca.SX.sym("psi")
    px        = ca.SX.sym("px")
    py        = ca.SX.sym("py")
    x = ca.vertcat(theta_f, omega_f, v_trl, omega_trl, psi, px, py)

    # inputs: [tau1, tau2]
    tau1 = ca.SX.sym("tau1")
    tau2 = ca.SX.sym("tau2")
    u = ca.vertcat(tau1, tau2)

    # parameters
    r_dw     = ca.SX.sym("r_dw")
    r_dwl    = ca.SX.sym("r_dwl")
    l_wb     = ca.SX.sym("l_wb")
    f_ric    = ca.SX.sym("f_ric")
    m_robot  = ca.SX.sym("m_robot")
    m_payload= ca.SX.sym("m_payload")
    g        = ca.SX.sym("g")
    I_zz     = ca.SX.sym("I_zz")
    m_pay    = ca.SX.sym("m_pay")
    l_lcom   = ca.SX.sym("l_lcom")
    I_pivot  = ca.SX.sym("I_pivot")

    p = ca.vertcat(r_dw, r_dwl, l_wb, f_ric, m_robot, m_payload, g, I_zz, m_pay, l_lcom, I_pivot)

    M = m_robot + m_payload
    I = I_zz + m_pay * (l_lcom**2)

    # smooth sign(v) for friction direction
    eps = 1e-3
    velocity_sigmoid = v_trl / ca.sqrt(v_trl**2 + eps)

    # connector:
    theta = theta_f
    Vf = (omega_trl * l_wb) * ca.sin(theta) + (v_trl * ca.cos(theta))

    # forces
    F_rect = (tau1 + tau2) * r_dwl * ca.cos(theta_f)
    F_rot  = (tau1 - tau2) * r_dwl * ca.cos(theta_f)

    # pivot dynamics (with pivot inertia)
    dd_theta_f = (2 * (r_dw**2) * (tau1 - tau2) - (Vf * ca.cos(theta_f) / l_wb)) / I_pivot

    # trailer dynamics
    alpha_trl = (F_rot * l_wb - f_ric * M * g * velocity_sigmoid) / I
    accel_trl = (F_rect       - f_ric * M * g * velocity_sigmoid) / M

    # global kinematics
    psi_dot = omega_trl
    px_dot  = v_trl * ca.cos(psi)
    py_dot  = v_trl * ca.sin(psi)

    xdot = ca.vertcat(
        omega_f,
        dd_theta_f,
        accel_trl,
        alpha_trl,
        psi_dot,
        px_dot,
        py_dot
    )

    # x_dot =A * f(x,u,p) + B * g (x,u,p) # dynamic system
    # y     =C * f(x,u,p)  # sensor data

    return ca.Function("f_p3000", [x, u, p], [xdot], ["x", "u", "p"], ["xdot"])


# -----------------------------
# 2) ZOH + RK4 discretization
# -----------------------------
def make_rk4_step(f, dt):
    x = ca.SX.sym("x", 7)
    u = ca.SX.sym("u", 2)
    p = ca.SX.sym("p", 11)

    k1 = f(x=x, u=u, p=p)["xdot"]
    k2 = f(x=x + dt/2 * k1, u=u, p=p)["xdot"]
    k3 = f(x=x + dt/2 * k2, u=u, p=p)["xdot"]
    k4 = f(x=x + dt    * k3, u=u, p=p)["xdot"]

    x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    return ca.Function("F_rk4", [x, u, p], [x_next], ["x", "u", "p"], ["x_next"])


# -----------------------------
# 3) Build a minimal MPC solver (multiple shooting)
# -----------------------------
def build_mpc_solver(F, N, dt, u_min, u_max):
    nx = 7
    nu = 2

    # Decision variables
    X = ca.SX.sym("X", nx, N+1)  # state trajectory
    U = ca.SX.sym("U", nu, N)    # input trajectory

    # Parameters to the NLP
    x0   = ca.SX.sym("x0", nx)
    xref = ca.SX.sym("xref", nx)      # constant reference over horizon
    p    = ca.SX.sym("p", 11)         # plant parameters

    # Weights
    Q = np.diag([1.0, 0.1, 2.0, 0.1, 0.2, 10.0, 10.0])
    R = np.diag([0.05, 0.05])
    QN = 5.0 * Q

    Q = ca.DM(Q)
    R = ca.DM(R)
    QN = ca.DM(QN)

    # Objective and constraints
    #cost = 0
    g = []          # equality constraints
    lbg = []
    ubg = []

    # initial condition constraint: X[:,0] = x0
    g.append(X[:, 0] - x0)
    lbg += [0]*nx
    ubg += [0]*nx

    # dynamics constraints + stage cost
    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        x_next = F(x=xk, u=uk, p=p)["x_next"]
        g.append(X[:, k+1] - x_next)
        lbg += [0]*nx
        ubg += [0]*nx

        e = xk - xref
        cost += ca.mtimes([e.T, Q, e]) + ca.mtimes([uk.T, R, uk])

    # terminal cost
    eN = X[:, N] - xref
    cost += ca.mtimes([eN.T, QN, eN])

    # Flatten decision variables
    w = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

    # Bounds on decision variables
    # States: unbounded (minimal). Inputs: bounded.
    w_lb = [-ca.inf] * (nx*(N+1)) + (u_min.tolist() * N)
    w_ub = [ ca.inf] * (nx*(N+1)) + (u_max.tolist() * N)

    # NLP
    nlp = {"x": w, "f": cost, "g": ca.vertcat(*g), "p": ca.vertcat(x0, xref, p)}

    opts = {
        "ipopt.print_level": 0,
        "print_time": 0,
        "ipopt.max_iter": 200,
    }
    solver = ca.nlpsol("mpc", "ipopt", nlp, opts)

    meta = {
        "nx": nx, "nu": nu, "N": N,
        "w_lb": np.array(w_lb, dtype=float),
        "w_ub": np.array(w_ub, dtype=float),
        "lbg": np.array(lbg, dtype=float),
        "ubg": np.array(ubg, dtype=float),
    }
    return solver, meta


def unpack_solution(sol, nx, nu, N):
    w_opt = np.array(sol["x"]).reshape(-1)
    X_opt = w_opt[: nx*(N+1)].reshape((N+1, nx))
    U_opt = w_opt[nx*(N+1):].reshape((N, nu))
    return X_opt, U_opt


# -----------------------------
# 4) Closed-loop run (minimal demo)
# -----------------------------
def main():
    # Plant and discretization
    f = make_p3000_plant()
    dt = 0.1
    F = make_rk4_step(f, dt)

    # Horizon
    N = 15  # 1.5 seconds

    # Input bounds
    tau_max = 5.0
    u_min = np.array([-tau_max, -tau_max], dtype=float)
    u_max = np.array([ tau_max,  tau_max], dtype=float)

    solver, meta = build_mpc_solver(F, N, dt, u_min, u_max)

    # Parameters
    params = {
        "r_dw":    0.10,
        "r_dwl":   0.10,
        "l_wb":    1.00,
        "f_ric":   0.10,
        "m_robot": 50.0,
        "m_payload":20.0,
        "g":       9.81,
        "I_zz":    10.0,
        "m_pay":   20.0,
        "l_lcom":  0.30,
        "I_pivot": 2.0,
    }
    p = np.array(list(params.values()), dtype=float)

    # Initial state: [theta_f, omega_f, v_trl, omega_trl, psi, px, py]
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    # constant reference (target position, and prefer some forward velocity)
    xref = x.copy()
    xref[5] = 2.0   # target px
    xref[6] = 1.0   # target py
    xref[2] = 0.5   # desired v_trl

    # Initial guesses (important for speed)
    nx, nu = meta["nx"], meta["nu"]
    w0 = np.zeros(nx*(N+1) + nu*N, dtype=float)

    # closed-loop sim length
    Tsim = 5.0
    steps = int(Tsim / dt)

    X_log = np.zeros((steps+1, nx))
    U_log = np.zeros((steps, nu))
    X_log[0] = x

    for tstep in range(steps):
        # NLP parameters vector: [x0; xref; p]
        nlp_p = np.concatenate([x, xref, p])

        sol = solver(
            x0=w0,
            lbx=meta["w_lb"],
            ubx=meta["w_ub"],
            lbg=meta["lbg"],
            ubg=meta["ubg"],
            p=nlp_p
        )

        X_opt, U_opt = unpack_solution(sol, nx, nu, N)

        u0 = U_opt[0]
        U_log[tstep] = u0

        # apply u0 to the plant (one step)
        x = np.array(F(x=x, u=u0, p=p)["x_next"]).reshape(-1)
        X_log[tstep+1] = x

        # warm start next solve: shift trajectories
        # shift X and U forward by one step
        X_shift = np.vstack([X_opt[1:], X_opt[-1:]])
        U_shift = np.vstack([U_opt[1:], U_opt[-1:]])
        w0 = np.concatenate([X_shift.reshape(-1), U_shift.reshape(-1)])

    # Minimal text output so you know it ran
    print("Final state:", X_log[-1])
    print("Final position (px,py):", X_log[-1, 5], X_log[-1, 6])
    print("First control applied:", U_log[0])


if __name__ == "__main__":
    main()
