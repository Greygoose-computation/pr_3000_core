import casadi as ca
print("Casadi version",ca.__version__)

x=ca.SX.sym("x")
f=ca.Function("f",[x],[x**2+1])

print("f(3) =",f(3))

