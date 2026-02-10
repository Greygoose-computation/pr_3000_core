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
import matplotlib.pyplot as plt


def make_p3000_plant():
    """
    States:
      x = [theta_f, omega_f, v_trl, omega_trl, theta_trl, px, py]
      x1 theta_f   : pivot angle                     ->  maps to state 2 integration
      x2 omega_f   : pivot angular rate              ->  maps to diff #1
      x3 v_trl     : trailer linear velocity         ->  maps to diff #3
      x4 omega_trl : trailer yaw rate                ->  maps to diff #2
      x5 theta_trl : trailer heading angle (global)  ->  maps to state 4 integration
      x6 px, py    : trailer position (global)       #   not in use

    Inputs:
      u = [tau1, tau2]  (left/right torques)

    Parameters (constants):
      p = [r_dw, r_dwl, l_wb, f_ric, m_robot, m_payload, g, I_zz, m_pay, l_lcom, I_pivot]
    """
    # ---- states ----
    theta_f   = ca.SX.sym("theta_f")
    omega_f   = ca.SX.sym("omega_f")
    v_trl     = ca.SX.sym("v_trl")
    omega_trl = ca.SX.sym("omega_trl")
    theta_trl = ca.SX.sym("theta_trl")

    x = ca.vertcat(theta_f, omega_f, v_trl, omega_trl, theta_trl)

    # ---- inputs ----
    tau1 = ca.SX.sym("tau1")
    tau2 = ca.SX.sym("tau2")
    u = ca.vertcat(tau1, tau2)

    # ---- parameters ----
    r_dw     = ca.SX.sym("r_dw")      # drive wheel radius for pivot dynamics term
    r_dwl    = ca.SX.sym("r_dwl")     # effective wheel radius/lever used for forces
    l_wb     = ca.SX.sym("l_wb")      # wheelbase / lever arm
    r_trl    = ca.SX.sym("r_trl")     # trailer wheel track for yaw friction lever arm
    f_ric    = ca.SX.sym("f_ric")     # friction coefficient
    m_robot  = ca.SX.sym("m_robot")   # mass of the robot body
    m_payload= ca.SX.sym("m_payload") # mass of the payload body
    g        = ca.SX.sym("g")         # gravity
    I_zz     = ca.SX.sym("I_zz")      # trailer yaw inertia
    l_lcom   = ca.SX.sym("l_lcom")    # payload COM offset lever
    I_pivot  = ca.SX.sym("I_pivot")   # steering/pivot inertia

    p = ca.vertcat(
        r_dw,
        r_dwl,
        l_wb,
        r_trl,
        f_ric,
        m_robot,
        m_payload,
        g,
        I_zz,
        l_lcom,
        I_pivot
    )

    # ---- helper terms ----
    M = m_robot + m_payload
    I = I_zz + m_payload * (l_lcom**2)

    # smooth sign for friction
    eps = 1e-3
    sig_v = v_trl / ca.sqrt(v_trl**2 + eps)
    sig_w = omega_trl / ca.sqrt(omega_trl**2 + eps)

    # ---- forces from inputs ----
    F_rect = ((tau1 + tau2) / r_dwl) * ca.cos(theta_f)
    F_rot  = ((tau1 - tau2) / r_dwl) * ca.cos(theta_f)

    # ---- trailer dynamics ----
    accel_trl = (F_rect - f_ric * M * g * sig_v) / M
    alpha_trl = (F_rot * l_wb - f_ric * M * g * r_trl * sig_w) / I

    # ---- connector dynamics ----
    theta = theta_f
    accel_f = accel_trl * ca.cos(theta) + (alpha_trl * l_wb) * ca.sin(theta)

    # ---- pivot dynamics ----
    dd_theta_f = (
        ((tau1 - tau2) / r_dwl) * (2 * r_dw)
        - accel_f * ca.sin(theta_f) / l_wb
    ) / I_pivot

    # ---- state derivatives ----
    theta_f_dot   = omega_f
    omega_f_dot   = dd_theta_f
    v_trl_dot     = accel_trl
    omega_trl_dot = alpha_trl
    theta_trl_dot = omega_trl

    xdot = ca.vertcat(theta_f_dot, omega_f_dot, v_trl_dot, omega_trl_dot, theta_trl_dot)

    f = ca.Function("f_p3000", [x, u, p], [xdot], ["x", "u", "p"], ["xdot"])
    return f


# ----------------------------------------------------------------------------------------------------------------------
# ZOH discretization with RK4
# ----------------------------------------------------------------------------------------------------------------------
def make_rk4_step(f, dt):
    x = ca.SX.sym("x", 5)
    u = ca.SX.sym("u", 2)
    p = ca.SX.sym("p", 11)

    k1 = f(x=x, u=u, p=p)["xdot"]
    k2 = f(x=x + dt/2 * k1, u=u, p=p)["xdot"]
    k3 = f(x=x + dt/2 * k2, u=u, p=p)["xdot"]
    k4 = f(x=x + dt    * k3, u=u, p=p)["xdot"]

    x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    return ca.Function("F_rk4", [x, u, p], [x_next], ["x", "u", "p"], ["x_next"])


# -----------------------------
# Build MPC controller
# -----------------------------
def build_mpc_solver(F, N, u_min, u_max):
    """
    Discrete-time MPC using multiple shooting:
      x_{k+1} = F(x_k, u_k, p)
    Decision vars: X[0..N], U[0..N-1]
    Params: x0, xref, p
    """
    nx = 5
    nu = 2

    X = ca.SX.sym("X", nx, N+1)
    U = ca.SX.sym("U", nu, N)

    x0   = ca.SX.sym("x0", nx)
    xref = ca.SX.sym("xref", nx)
    p    = ca.SX.sym("p", 11)

    Q = ca.DM(np.diag([
        5.0,  # theta_f
        5.0,  # omega_f
        5.0,  # v_trl
        5.0,  # omega_trl
        5.0  # theta_trl
    ]))

    R = ca.DM(np.diag([0.05, 0.05]))

    QN = ca.DM(np.diag([
        1.0 * 5.0,  # theta_f
        1.0 * 5.0,  # omega_f
        50.0 * 5.0,  # v_trl
        1.0 * 5.0,  # omega_trl
        50.0 * 5.0  # theta_trl
    ]))

    cost = 0
    g = []

    g.append(X[:, 0] - x0)

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        x_next = F(x=xk, u=uk, p=p)["x_next"]
        g.append(X[:, k+1] - x_next)

        e = xk - xref
        cost += ca.mtimes([e.T, Q, e]) + ca.mtimes([uk.T, R, uk])

    eN = X[:, N] - xref
    cost += ca.mtimes([eN.T, QN, eN])

    w = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

    nX = nx * (N + 1)

    w_lb = [-ca.inf] * nX + list(u_min) * N
    w_ub = [ ca.inf] * nX + list(u_max) * N

    g_flat = ca.vertcat(*g)
    lbg = [0.0] * int(g_flat.size1())
    ubg = [0.0] * int(g_flat.size1())

    nlp = {"x": w, "f": cost, "g": g_flat, "p": ca.vertcat(x0, xref, p)}

    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 200}
    solver = ca.nlpsol("mpc", "ipopt", nlp, opts)

    meta = {
        "nx": nx, "nu": nu, "N": N,
        "w_lb": np.array(w_lb, dtype=float),
        "w_ub": np.array(w_ub, dtype=float),
        "lbg": np.array(lbg, dtype=float),
        "ubg": np.array(ubg, dtype=float),
        "nX": nX,
    }
    return solver, meta


def unpack_solution(sol, nx, nu, N):
    w_opt = np.array(sol["x"]).reshape(-1)

    nX = nx*(N+1)
    X_opt = w_opt[:nX].reshape((nx, N+1), order="F")
    U_opt = w_opt[nX:].reshape((nu, N), order="F")

    return X_opt, U_opt


# -----------------------------
# Plotting
# -----------------------------
def plot_mpc_results(dt, X_log, U_log, xref):
    """
    X_log: (steps+1, nx)
    U_log: (steps, nu)
    xref : (nx,)
    """
    steps = U_log.shape[0]
    tX = np.arange(steps + 1) * dt
    tU = np.arange(steps) * dt

    state_names = ["theta_f", "omega_f", "v_trl", "omega_trl", "theta_trl"]

    # States vs reference
    fig1, axs = plt.subplots(5, 1, sharex=True, figsize=(10, 10))
    for i in range(5):
        axs[i].plot(tX, X_log[:, i], label="x")
        axs[i].plot(tX, np.full_like(tX, xref[i]), "--", label="xref")
        axs[i].set_ylabel(state_names[i])
        axs[i].grid(True)
        if i == 0:
            axs[i].legend(loc="best")
    axs[-1].set_xlabel("time [s]")
    fig1.suptitle("State trajectories vs reference")
    plt.tight_layout()

    # Inputs
    fig2, ax = plt.subplots(1, 1, figsize=(10, 4))
    ax.step(tU, U_log[:, 0], where="post", label="tau1")
    ax.step(tU, U_log[:, 1], where="post", label="tau2")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("torque [Nm]")
    ax.grid(True)
    ax.legend(loc="best")
    fig2.suptitle("Input trajectory")
    plt.tight_layout()

    plt.show()


def main():
    f = make_p3000_plant()
    dt = 0.1
    F = make_rk4_step(f, dt)

    N = 5
    tau_max = 7.5
    u_min = np.array([-tau_max, -tau_max], dtype=float)
    u_max = np.array([ tau_max,  tau_max], dtype=float)

    solver, meta = build_mpc_solver(F, N, u_min, u_max)

    # Parameters
    params = {
        "r_dw":     0.10,
        "r_dwl":    0.10,
        "l_wb":     1.00,
        "r_trl":    0.60,
        "f_ric":    0.10,
        "m_robot":  50.0,
        "m_payload":20.0,
        "g":        9.81,
        "I_zz":     10.0,
        "l_lcom":   0.30,
        "I_pivot":  2.0,
    }
    p = np.array(list(params.values()), dtype=float)

    # theta_f , omega_f , v_trl , omega_trl , theta_trl

    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    xref = np.array([0.0, 0.0, 1.0, 0.0, 10.0], dtype=float)

    nx, nu = meta["nx"], meta["nu"]
    w0 = np.zeros(nx*(N+1) + nu*N, dtype=float)

    Tsim = 10.0
    steps = int(Tsim / dt)

    X_log = np.zeros((steps+1, nx))
    U_log = np.zeros((steps, nu))
    X_log[0] = x

    for tstep in range(steps):
        nlp_p = np.concatenate([x, xref, p])

        sol = solver(
            x0=w0,
            lbx=meta["w_lb"], ubx=meta["w_ub"],
            lbg=meta["lbg"], ubg=meta["ubg"],
            p=nlp_p
        )

        X_opt, U_opt = unpack_solution(sol, nx, nu, N)

        u0 = U_opt[:, 0]
        U_log[tstep] = u0

        x = np.array(F(x=x, u=u0, p=p)["x_next"]).reshape(-1)
        X_log[tstep+1] = x

        # warm start
        X_shift = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])
        U_shift = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])

        w0 = np.concatenate([X_shift.reshape(-1, order="F"), U_shift.reshape(-1, order="F")])

    print("Final state:", X_log[-1])
    print("First control applied:", U_log[0])

    # Plot results
    plot_mpc_results(dt, X_log, U_log, xref)


if __name__ == "__main__":
    main()
