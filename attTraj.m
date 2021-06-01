function [sigM,wm_dot,Wm] = attTraj(m,U,g,psid,Wm,sigM,dt)
gv = [0;0;g];
Finertial = m*(gv-U);
unitF = Finertial/norm(Finertial);
% force vector orientation in body frame(straight above z)
uvb = [0;0;1];
numerator = cross(unitF,uvb);
denominator = (norm(uvb))^2+(uvb')*unitF+norm(uvb)*(sqrt(2*(norm(uvb))^2+(uvb')*unitF));
sigD = numerator/denominator;
% incorporating psid
quat = [cos(psid/2);0;0;sin(psid/2)];
sig_psi = quatTOmrp(quat);
sigD = addMRP(sigD,sig_psi);
% a nonlinear reference model is employed to achieve the time-scale separation between the inner and outer loops.
A1 = [100 0 0;0 100 0;0 0 25];
A2 = [20 0 0; 0 20 0;0 0 10];
er_sig=addMRP(sigD,-sigM);
wm_dot=-A1*(er_sig)-A2*(Wm);
Wm=Wm+wm_dot*dt;
sigM_dot=sdot(sigM)*Wm;
sigM=sigM+sigM_dot*dt;
if norm(sigM)>1
    sigM=switchMRP(sigM);
end 
end
function sd=sdot(x)
sd=(1/4)*((1-x'*x)*eye(3)+2*skew(x)+2*(x)*x');
end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end
