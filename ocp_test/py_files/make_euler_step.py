import casadi as ca

#-descretization

def make_euler_step(f,dt:float):
    """
    Discrete-time step:
    x_next = x + dt*f(x,u)
    :param f:
    :param dt:
    :return: CasADI function F(x,u) -> x_next
    """

    x= ca.SX.sym("x",3)
    u= ca.SX.sym("u",2)
    x_next = x + dt*f(x,u)
    f=ca.Function("F_euler",[x,u],[x_next],["x","u"],["x_next"])
    return f
