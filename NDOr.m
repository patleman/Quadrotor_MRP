function [vr,dr]=NDOr(vr,dr,I,Ur,dt,w)
Ld=3*eye(3,3);
dventa=-Ld*(dr)+Ld*(cross(w,I*w)-Ur);
vr=vr+dventa*dt;
dr=vr+Ld*I*w;

end