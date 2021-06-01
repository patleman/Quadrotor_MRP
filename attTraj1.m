function [Fbody,sigD,sigC,sigC_dot,wC_dot,wC] = attTraj1(m,U,g,psid,sigC,sigC_dot,dt)
Fbody= m*norm(U);

uv=(m/Fbody).*U;% unit vector

e3=[0;0;1];
%denominator = (norm(uv)^2)+(uv')*e3+norm(uv)*(sqrt(2*(norm(uv))^2+(uv')*e3));
%sigD = (1/denominator)*[-uv(2);uv(1);0];
denominator = (norm(uv)^2)+(uv')*e3+norm(uv)*(sqrt(2*(norm(uv)^2+(uv')*e3)));
sigD = -(1/denominator)*cross(uv,e3);
%%%%%%  adding psid
%q=[cos(psid*pi/360);0;0;sin(psid*pi/360)];
%siga=[q(2)/(1+q(1));q(3)/(1+q(1));q(4)/(1+q(1))];

%sigD(3)=siga(3);


%%%%%%
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
