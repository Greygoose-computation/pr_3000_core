#--This script contains the plant model of the P3000 platform

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

#-p3000 model for pivot body dynamics

""" 
    # Friction parameters 
    y = 0.3 * ( ( exp( 5*(5*sqrt(x.^2) - 0.2) ) - exp( -10*(5*sqrt(x.^2) - 0.5) ) ) ./ ...
            ( exp(10*(5*sqrt(x.^2) - 0.5)) + exp( -10*(5*sqrt(x.^2) - 0.5) ) ) ...
          + 1.0 );
    
    where y = effective friction coeff.
          x = velcoity of the body 
    # pivot body input dynamics 
    dd_theta_f=(((tau1-tau2)/r_dwl)*2*(r_dw))/(I_pivot)-(accel_f*sin(theta_f)/l_wb) : angular acceleration of the steering pivot  --> diff #1
    
    # trailing body input dynamics from the leading steering pivot body 
    F_rect=(tau1+tau2)/(r_dwl)*cos(theta_f) : Force_rectilinear    # force equations 
    F_rot=(tau1-tau2)/(r_dwl)*cos(theta_f) : Force_rot             # force equations 
    
    # trailer body dynamics :  
    alpha_trl=((F_rot*l_wb)-(f_ric*(m_robot+m_payload)*g*(velocity_sigmoid))*(r_trl))/(I_zz+mass_pay*(l_lcom)**2) : angular acceleration around rear rollers  --> diff #2
    accel_trl=((F_rect)-(f_ric*(m_robot+m_payload)*g*(velocity_sigmoid)))/(m_robot+m_payload) : linear acceleration of the body)  --> diff #3 
    
    # connector dynamics :
    accel_f=(accel_trl*cos(theta)+(alpha_trl*l_wb*sin(theta)))
    
"""
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

    #------------------------
    # Plant order
    # x_dot = f(X,U)
    # y     = g(X,U)

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

    def fric_eff(v):
        x = ca.fabs(v)

        mu = 0.05 * (
                (ca.exp(5 * (5 * x - 0.2)) - ca.exp(-10 * (5 * x - 0.5))) /
                (ca.exp(10 * (5 * x - 0.5)) + ca.exp(-10 * (5 * x - 0.5)))
                + 1.0
        )

        return mu

    # ---- forces from inputs ----
    F_rect = ((tau1 + tau2) / r_dwl) * ca.cos(theta_f)
    F_rot = ((tau1 - tau2) / r_dwl) * ca.cos(theta_f)

    # ---- trailer dynamics ----
    accel_trl = (F_rect - fric_eff(v_trl) * M * g ) / M
    alpha_trl = (F_rot * l_wb - fric_eff(v_trl) * M * g * r_trl) / I

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


# ----------------------------------------------------------------------------------------------------------------------
# main loop (updated for 5-state model + 11 params incl. r_trl)
# ----------------------------------------------------------------------------------------------------------------------
def main():
    f = make_p3000_plant()
    dt = 0.05
    F = make_rk4_step(f, dt)

    # Parameter order MUST match:
    # p = [r_dw, r_dwl, l_wb, r_trl, f_ric, m_robot, m_payload, g, I_zz, l_lcom, I_pivot]
    params = {
        "r_dw":     0.10,
        "r_dwl":    0.10,
        "l_wb":     1.00,
        "r_trl":    0.60,   # <-- ADD: distance between trailer wheels (track width)
        "f_ric":    0.10,
        "m_robot":  50.0,
        "m_payload":20.0,
        "g":        9.81,
        "I_zz":     10.0,
        "l_lcom":   0.30,
        "I_pivot":  2.0,
    }
    p = np.array(list(params.values()), dtype=float)

    # ---- initial state ----
    # x = [theta_f, omega_f, v_trl, omega_trl, theta_trl]
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    # ---- inputs ----
    # u = [tau1, tau2]
    u = np.array([10.0, 10.0], dtype=float)

    T = 100.0
    N = int(T / dt)
    t = np.linspace(0.0, T, N + 1)

    X = np.zeros((N + 1, 5))
    U = np.zeros((N, 2))

    X[0] = x

    for k in range(N):
        U[k] = u
        x = np.array(F(x=x, u=u, p=p)["x_next"]).reshape(-1)  # (5,)
        X[k + 1] = x

    # ---- plots ----
    # Velocities
    plt.figure()
    plt.plot(t, X[:, 2], label="v_trl")
    plt.plot(t, X[:, 3], label="omega_trl")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("value")
    plt.title("Trailer velocities (open-loop)")
    plt.legend()

    # Pivot
    plt.figure()
    plt.plot(t, X[:, 0], label="theta_f")
    plt.plot(t, X[:, 1], label="omega_f")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("value")
    plt.title("Pivot states (open-loop)")
    plt.legend()

    # Trailer heading
    plt.figure()
    plt.plot(t, X[:, 4], label="theta_trl")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("rad")
    plt.title("Trailer heading (open-loop)")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()

# controller building


