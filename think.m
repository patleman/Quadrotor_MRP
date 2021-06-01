clc
clear
t=0:0.1:pi;
a=size(t);
d=zeros(1,32);
i=1;
for theta=0:0.1:pi
%fx=sin(theta/2)/(1+cos(theta/2));
q=[cos(theta/2);0;sin(theta/2);0];
sig=[0;q(3)/(1+q(1));0];

nom=norm(sig)^2;
d(i)=nom;
i=i+1;
end
%plot(t,d,'--r')
% checking 180
q=[cos(120*pi/360);(1/sqrt(3))*sin(0*pi/360);sin(120*pi/360);(1/sqrt(1))*sin(0*pi/360)];
sig=[q(2)/(1+q(1));q(3)/(1+q(1));q(4)/(1+q(1))];
%q=[cos(2.6/2);0;sin(2.6/2);0];
%sig=[0;q(3)/(1+q(1));0];
norm(sig);
norm(sig)^2;
changedCordi=mrpTOdcm(sig)*[1;0;0]

uv=[1;0;1];
k=norm(uv);
uv=uv/norm(uv);

e3=[0;0;1];
denominator = (norm(uv)^2)+(uv')*e3+norm(uv)*(sqrt(2*(norm(uv)^2)+(uv')*e3));

sigD = (1/denominator)*[-uv(2);uv(1);0];
changedCordi=(mrpTOdcm(sigD)')*e3;%
%k*changedCordi
denominator = (norm(uv)^2)+(uv')*e3+norm(uv)*(sqrt(2*(norm(uv)^2+(uv')*e3)));
sigD = -(1/denominator)*cross(uv,e3);
changedCordi=(mrpTOdcm(sigD)')*e3;%
%k*changedCordi
d=0.1785;
Motor_cordsB=[-d,d,d,-d;
               d,d,-d,-d;
    0,0,0,0];
Motor_cordsI=((mrpTOdcm(sig))')*Motor_cordsB;
minZ=Motor_cordsI(3,1);% refereing to motor 1
m1=1;
mat=zeros(4,4);
for t=2:4
if Motor_cordsI(3,t)<=minZ
    if  Motor_cordsI(3,t)==minZ
        mat(m1,t)=1;
    else
       minZ=Motor_cordsI(3,t); 
       m1=t;
       if m1>=2
        m2=m1-1;
        else
        m2=m1+1;
       end
    end
end
end
if minZ==0 
   
    m1=1;
    
end
if m1~=4
mf=m1+1;
else
    mf=1;
end
if m1~=1
mb=m1-1;
else
    mb=4;
end
if Motor_cordsI(3,mf)<Motor_cordsI(3,mb)
    m2=mf;
else
    m2=mb;
end
f_v=zeros(4,1);
f_v(m1)=5.025;
f_v(m2)=5.025;
f_v
b=[0;0;1];
a=(mrpTOdcm(sig)')*b;%if degAngle>=39 && firsttime==0 &&  fex==0 % 
degAngle =(180/pi)* atan2(norm(cross(b,a)), dot(b,a));
%%%%%%%%%%%%%%%%%%%%%
phi=60;
thi=45;
q0=[cos(phi*pi/360);0;0;sin(phi*pi/360)];

sig0=[q0(2)/(1+q0(1));q0(3)/(1+q0(1));q0(4)/(1+q0(1))];
q1=[cos(thi*pi/360);0;0;sin(thi*pi/360)];

sig1=[q1(2)/(1+q1(1));q1(3)/(1+q1(1));q1(4)/(1+q1(1))];
addMRP(sig1,addMRP(sig0,-sig1))
%%%%%%%%%%%
s1=[0;0;1];
%s1=switchMRP(s1);
s2=[0;0;1];
numerator = (1-(norm(s2)^2))*s1+(1-(norm(s1)^2))*s2-2*cross(s1,s2);
denominator = 1+(norm(s1)^2)*(norm(s2)^2)-2*(s1')*s2
c1=numerator/denominator;
%%%%%%%%%%%%%%%%%%
 