function [v,d]=NDOt(v,d,m,U,dt,xr_dot,x_dot,xrd_dot)
L=4;
v_dot = -L*(-9.8*[0;0;1]+U+(d/m));%-xrd_dot);
v = v+v_dot*dt;
%size(v)
const=L;
d = m*(v+const*(x_dot));%-xr_dot);

end