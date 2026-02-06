function model = unicycle_model()
import casadi.*

%states 
px=SX.sym('px');
py=SX.sym('py');
th=SX.sym('th');
x=[px;py;th]; % state vector

%control inpiuts 
v = SX.sym('v');
w = SX.sym('w');
u = [v,w];

%dynamics 
xdot = [v*cos(th);
        v*sin(th);
        w];
model.x = x;
model.u = u;
model.xdot=SX.sym('xdot',3,1);
model.f_expl_expr=xdot;

model.name='unicycle';
end

