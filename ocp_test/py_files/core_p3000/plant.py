#--This script contains the plant model of the P3000 platform

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

#-p3000 model for pivot body dynamics

"""
    # pivot body input dynamics 
    dd_theta_f=(tau1-tau2)*2*(r_dw)**2-(Vf*cos(theta_f)/l_wb) : angular acceleration of the steering pivot  --> diff #1
    
    # trailing body input dynamics from the leading steering pivot body 
    F_rect=(tau1+tau2)*(r_dwl)*cos(theta_f) : Force_rectilinear    # force equations 
    F_rot=(tau1-tau2)*(r_dwl)*cos(theta_f) : Force_rot             # force equations 
    
    # trailer body dynamics 
    alpha_trl=((F_rot*l_wb)-(f_ric*(m_robot+m_payload)*g*(velocity_sigmoid)))/(I_zz+mass_pay*(l_lcom)**2) : angular acceleration around rear rollers  --> diff #2
    accel_trl=(F_rect)-(f_ric(m_robot+m_payload)*g*(velocity_sigmoid))/(m_robot+m_payload) : linear acceleration of the body)  --> diff #3 
    
    connector dynamics :
    Vf=(omega_trl*l_wb)*sin(theta) +(vel_trl*cos(theta))
    
    
"""
def make_p3000_plant():
    """
    States:
      x = [theta_f, omega_f, v_trl, omega_trl, psi, px, py]
        theta_f   : pivot angle                     ->  maps to diff #2
        omega_f   : pivot angular rate              ->  maps to diff #1
        v_trl     : trailer linear velocity         ->  maps to diff #2
        omega_trl : trailer yaw rate                ->  maps to diff #2
        psi       : trailer heading angle (global)  # not in use
        px, py    : trailer position (global)       # not in use

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
    psi       = ca.SX.sym("psi")
    px        = ca.SX.sym("px")
    py        = ca.SX.sym("py")

    x = ca.vertcat(theta_f, omega_f, v_trl, omega_trl, psi, px, py)

    # ---- inputs ----
    tau1 = ca.SX.sym("tau1")
    tau2 = ca.SX.sym("tau2")
    u = ca.vertcat(tau1, tau2)

    # ---- parameters ----
    r_dw     = ca.SX.sym("r_dw")      # drive wheel radius for pivot dynamics term (your notation)
    r_dwl    = ca.SX.sym("r_dwl")     # effective wheel radius/lever used for forces
    l_wb     = ca.SX.sym("l_wb")      # wheelbase / lever arm
    f_ric    = ca.SX.sym("f_ric")     # friction coeff
    m_robot  = ca.SX.sym("m_robot")
    m_payload= ca.SX.sym("m_payload")
    g        = ca.SX.sym("g")
    I_zz     = ca.SX.sym("I_zz")      # trailer yaw inertia
    m_pay    = ca.SX.sym("m_pay")     # payload mass used in inertia augmentation
    l_lcom   = ca.SX.sym("l_lcom")    # payload COM offset lever
    I_pivot  = ca.SX.sym("I_pivot")   # steering/pivot inertia

    p = ca.vertcat(r_dw, r_dwl, l_wb, f_ric, m_robot, m_payload, g, I_zz, m_pay, l_lcom, I_pivot)

    # ---- helper terms ----
    M = m_robot + m_payload
    I = I_zz + m_pay * (l_lcom**2)

    # smoothening for friction
    eps = 1e-3
    velocity_sigmoid = v_trl / ca.sqrt(v_trl**2 + eps)

    # ---- connector dynamics ----

    theta = theta_f
    Vf = (omega_trl * l_wb) * ca.sin(theta) + (v_trl * ca.cos(theta))

    # ---- forces from inputs ----
    F_rect = (tau1 + tau2) * r_dwl * ca.cos(theta_f)
    F_rot  = (tau1 - tau2) * r_dwl * ca.cos(theta_f)

    # ---- pivot dynamics (now divided by pivot inertia) ----
    dd_theta_f = (2 * (r_dw**2) * (tau1 - tau2) - (Vf * ca.cos(theta_f) / l_wb)) / I_pivot

    # ---- trailer dynamics ----
    alpha_trl = (F_rot * l_wb - f_ric * M * g * velocity_sigmoid) / I
    accel_trl = (F_rect       - f_ric * M * g * velocity_sigmoid) / M

    # ---- global kinematics for trailer pose ----
    psi_dot = omega_trl
    px_dot  = v_trl * ca.cos(psi)
    py_dot  = v_trl * ca.sin(psi)

    # ---- state derivatives ----
    xdot = ca.vertcat(
        omega_f,
        dd_theta_f,
        accel_trl,
        alpha_trl,
        psi_dot,
        px_dot,
        py_dot
    )

    f = ca.Function("f_p3000", [x, u, p], [xdot], ["x", "u", "p"], ["xdot"])
    return f


# -----------------------------
# 2) ZOH discretization with RK4:
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
    F = ca.Function("F_rk4", [x, u, p], [x_next], ["x", "u", "p"], ["x_next"])
    return F



def main():
    f = make_p3000_plant()
    dt = 0.05
    F = make_rk4_step(f, dt)


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

    # ---- initial state ----
    # x = [theta_f, omega_f, v_trl, omega_trl, psi, px, py]
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    # ---- inputs (constant example) ----
    # tau1, tau2
    u = np.array([2.0, 1.0], dtype=float)

    T = 10.0
    N = int(T / dt)

    X = np.zeros((N + 1, 7))
    U = np.zeros((N, 2))
    t = np.linspace(0.0, T, N + 1)

    X[0] = x

    for k in range(N):
        U[k] = u
        x = np.array(F(x=x, u=u, p=p)["x_next"]).reshape(-1)  # (7,)
        X[k + 1] = x

"""
    # ---- plots ----
    # Trajectory
    plt.figure()
    plt.plot(X[:, 5], X[:, 6], linewidth=2)
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("px [m]")
    plt.ylabel("py [m]")
    plt.title("Trailer trajectory (open-loop)")

    # Velocities
    plt.figure()
    plt.plot(t, X[:, 2], label="v_trl")
    plt.plot(t, X[:, 3], label="omega_trl")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("value")
    plt.title("Trailer velocities")
    plt.legend()

    # Pivot
    plt.figure()
    plt.plot(t, X[:, 0], label="theta_f")
    plt.plot(t, X[:, 1], label="omega_f")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("value")
    plt.title("Pivot states")
    plt.legend()

    plt.show()

"""

if __name__ == "__main__":
    main()

# controller building


