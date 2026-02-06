import casadi as ca
import numpy as np

#-plant model continues

def make_unicycle():
    """
    continues-time unicycle model:
    x = [px, py, theta]
    u = [v, w]
    xdot = []
    :return:
    """
    x=ca.SX.sym("x",3)  # [px,py,th]
    u=ca.SX.sym("u",2)  # [v,w]

    px,py,th=x[0],x[1],x[2]
    v,w=u[0],u[1]

    xdot = ca.vertcat(v*ca.cos(th),v*ca.sin(th),w)
    f = ca.Function("f",[x,u],[xdot],["x","u"],["xdot"])
    return f