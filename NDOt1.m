function [z,d,t1,t2]=NDOt1(U,m,t1,t2,dt,z,x_dot)
gamma0=blkdiag([0.0003,0.0003,0.0003]);
gamma1=blkdiag([0.003,0.003,0.003]);
gamma2=blkdiag([0.03,0.03,0.03]);
t0=x_dot-z;
t1=t1+t0;
t2=t2+t1;
z_dot = (-9.8*[0;0;1]+U)+gamma0*t0+gamma1*t1+gamma2*t2;
z=z+dt*z_dot;
d=gamma0*t0+gamma1*t1+gamma2*t2;
d
end