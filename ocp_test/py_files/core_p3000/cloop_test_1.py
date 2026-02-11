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
    #px        = ca.SX.sym("px")  ->  let upper control handle pos
    #py        = ca.SX.sym("py")

    x = ca.vertcat(theta_f, omega_f, v_trl, omega_trl, theta_trl)

    # ---- inputs ----
    tau1 = ca.SX.sym("tau1")
    tau2 = ca.SX.sym("tau2")
    u = ca.vertcat(tau1, tau2)

    # ---- parameters ----
    r_dw     = ca.SX.sym("r_dw")      # drive wheel radius for pivot dynamics term (your notation)
    r_dwl    = ca.SX.sym("r_dwl")     # effective wheel radius/lever used for forces
    l_wb     = ca.SX.sym("l_wb")      # wheelbase / lever arm
    r_trl    = ca.SX.sym("r_trl")
    f_ric    = ca.SX.sym("f_ric")     # friction coefficient
    m_robot  = ca.SX.sym("m_robot")   # mass of the robot body
    m_payload= ca.SX.sym("m_payload") # mass of the payload body
    g        = ca.SX.sym("g")         # gravity
    I_zz     = ca.SX.sym("I_zz")      # trailer yaw inertia
    l_lcom   = ca.SX.sym("l_lcom")    # payload COM offset lever
    I_pivot  = ca.SX.sym("I_pivot")   # steering/pivot inertia

    p = ca.vertcat(
        r_dw,  # drive wheel radius (pivot actuation)
        r_dwl,  # wheel radius for force conversion
        l_wb,  # wheelbase / lever arm
        r_trl,  # trailer wheel track (yaw friction lever arm)  <-- ADD THIS
        f_ric,  # friction coefficient
        m_robot,  # robot mass
        m_payload,  # payload mass
        g,  # gravity
        I_zz,  # trailer yaw inertia
        l_lcom,  # payload COM offset
        I_pivot  # pivot inertia
    )


    # ---- helper terms ----
    M = m_robot + m_payload
    I = I_zz + m_payload * (l_lcom**2)    # rotational inertia around rear rollers when the force is from front pivot

    # smoothening for friction        !!!review this before freezing this !!!

    # TODO
    eps = 1e-3
    sig_v = v_trl / ca.sqrt(v_trl ** 2 + eps)          # ~sign(v_trl)
    sig_w = omega_trl / ca.sqrt(omega_trl ** 2 + eps)  # ~sign(omega_trl)

    # ---- forces from inputs ----
    F_rect = ((tau1 + tau2) / r_dwl) * ca.cos(theta_f)
    F_rot = ((tau1 - tau2) / r_dwl) * ca.cos(theta_f)

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

    # ---- global kinematics for trailer pose ----
    theta_trl_dot = omega_trl

    # ---- state derivatives ----
    theta_f_dot = omega_f
    omega_f_dot = dd_theta_f
    v_trl_dot = accel_trl
    omega_trl_dot = alpha_trl
    theta_trl_dot = omega_trl

    xdot = ca.vertcat(
        theta_f_dot,
        omega_f_dot,
        v_trl_dot,
        omega_trl_dot,
        theta_trl_dot
    )

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
import casadi as ca
import numpy as np

def build_mpc_solver(F, N, u_min, u_max):
    """
    Discrete-time MPC using multiple shooting:
      x_{k+1} = F(x_k, u_k, p)
    Decision vars: X[0..N], U[0..N-1]  ->  predicted states
    Params: x0, xref, p
    """
    nx = 5
    nu = 2

    # Decision variables
    X = ca.SX.sym("X", nx, N+1)
    U = ca.SX.sym("U", nu, N)

    # Parameters to the NLP
    x0   = ca.SX.sym("x0", nx)
    xref = ca.SX.sym("xref", nx)
    p    = ca.SX.sym("p", 11)

    # Weights
    # x = [theta_f, omega_f, v_trl, omega_trl, theta_trl]
    Q  = ca.DM(np.diag([5.0, 5.0, 5.0, 5.0, 5.0]))
    R  = ca.DM(np.diag([0.05, 0.05]))
    QN = 15.0 * Q

    cost = 0
    g = []   # need to manage the memeory here for acados transtion ,

    # Initial condition constraint
    g.append(X[:, 0] - x0)  # don't heap this

    # Dynamics + stage cost
    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        x_next = F(x=xk, u=uk, p=p)["x_next"]
        g.append(X[:, k+1] - x_next)

        e = xk - xref
        cost += ca.mtimes([e.T, Q, e]) + ca.mtimes([uk.T, R, uk])

    # Terminal cost
    eN = X[:, N] - xref
    cost += ca.mtimes([eN.T, QN, eN])

    # Flatten decision variables
    w = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

    # Bounds on decision variables
    # States unbounded; inputs bounded each stage
    nX = nx * (N + 1)
    nU = nu * N

    w_lb = [-ca.inf] * nX + list(u_min) * N
    w_ub = [ ca.inf] * nX + list(u_max) * N

    # Equality constraint bounds: all zeros
    g_flat = ca.vertcat(*g)
    lbg = [0.0] * int(g_flat.size1())
    ubg = [0.0] * int(g_flat.size1())

    # NLP
    nlp = {
        "x": w,
        "f": cost,
        "g": g_flat,
        "p": ca.vertcat(x0, xref, p),
    }

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
        "nX": nX, "nU": nU
    }
    return solver, meta


def unpack_solution(sol, nx, nu, N):
    w_opt = np.array(sol["x"]).reshape(-1)

    nX = nx*(N+1)
    X_opt = w_opt[:nX].reshape((nx, N+1), order="F")   # (nx, N+1)
    U_opt = w_opt[nX:].reshape((nu, N), order="F")     # (nu, N)

    return X_opt, U_opt


def main():
    f = make_p3000_plant()
    dt = 0.1
    F = make_rk4_step(f, dt)

    N = 50
    tau_max = 5.0
    u_min = np.array([-tau_max, -tau_max], dtype=float)
    u_max = np.array([ tau_max,  tau_max], dtype=float)

    solver, meta = build_mpc_solver(F, N, u_min, u_max)   #

    # Parameters
    # [r_dw, r_dwl, l_wb, r_trl, f_ric, m_robot, m_payload, g, I_zz, l_lcom, I_pivot]
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

    # 5-state initial condition
    # x = [theta_f, omega_f, v_trl, omega_trl, theta_trl]
    x = np.array([0.1, 0.0, 0.2, 0.0, 0.0], dtype=float)

    # 5-state reference
    xref = np.array([2.0, 0.0, 2.0, 0.0, 0.0], dtype=float)

    nx, nu = meta["nx"], meta["nu"]
    w0 = np.zeros(nx*(N+1) + nu*N, dtype=float)

    Tsim = 5.0
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

        u0 = U_opt[:, 0]          # <-- (nu,)
        U_log[tstep] = u0

        x = np.array(F(x=x, u=u0, p=p)["x_next"]).reshape(-1)
        X_log[tstep+1] = x

        # warm start (shift, keep last)
        X_shift = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])   # (nx, N+1)
        U_shift = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])   # (nu, N)

        w0 = np.concatenate([
            X_shift.reshape(-1, order="F"),
            U_shift.reshape(-1, order="F")
        ])

    print("Final state:", X_log[-1])
    print("First control applied:", U_log[0])
# -------------------------------------------------
# Run MPC script
# -------------------------------------------------
if __name__ == "__main__":
    main()
