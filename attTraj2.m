function [Fbody,sigD,sigC,sigC_dot,wC_dot,wC] = attTraj2(m,U,g,psid,sigC,sigC_dot,dt,t,Ur)
d=0.1785;% distance betweeen centre and line connecting two adjacent motors
%c=0.08;
c=(2.423*10^-7)/(8.050*10^-6);
A=[d d -d -d;d -d -d d;c -c c -c];
b=[Ur(1); Ur(2) ;Ur(3)];
M = [A b];
R = rref(M);
f4=R(3,5);
f1=f4*R(1,4)+R(1,5);
f2=f4*R(2,4)+R(2,5);
f3=f4*R(3,4)+R(3,5);
Fbody= m*norm(U);

uv=(m/Fbody).*U;
e3=[0;0;1];
denominator = (norm(uv)^2)+(uv')*e3+norm(uv)*(sqrt(2*(norm(uv)^2)+(uv')*e3));

sigD = (1/denominator)*[-uv(2);uv(1);0];
Fbody=f1+f2+f3+f4;

%sigD = [cos(t/1.5);0;0];

zeta=.707;
wn=12;
sigC=sigC+dt*sigC_dot;

sigCd_dot=-2*zeta*wn*sigC_dot-(wn^2)*(sigC-sigD);

sigC_dot=sigC_dot + dt*sigCd_dot;

wC=sdot(sigC)\sigC_dot;

wC_dot=wCd(sigC,sigC_dot,sigCd_dot);
end
function sd=sdot(x)
sd=(1/4)*((1-x'*x)*eye(3)+2*skew(x)+2*(x)*x');

end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end
