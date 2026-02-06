import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from make_euler_step import make_euler_step
from unicycle_casadi import make_unicycle


# simulation ----------------------------
dt=0.1
T=7.0
N=int(T/dt)

f=make_unicycle()
F=make_euler_step(f,dt)

x=np.array([0.0,0.0,0.0])
u=np.array([0.6,0.6])

traj = np.zeros((N+1, 3))
traj[0] = x

for k in range(N):
    x = np.array(F(x=x, u=u)["x_next"]).squeeze()
    traj[k+1] = x

# ---------- Plot ----------
plt.figure()
plt.plot(traj[:, 0], traj[:, 1], linewidth=2)
plt.axis("equal")
plt.grid(True)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Unicycle open-loop trajectory (ZOH + RK4)")
plt.show()