function [x,x_dot]=TranslationalDynamics(U,x_dot,x,dreal,m,dt)
vdot=-[0;0;9.8]+U+(dreal/m);

x=x+x_dot*dt;

x_dot=x_dot+vdot*dt;

end