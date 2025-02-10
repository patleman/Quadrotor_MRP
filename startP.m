function [ I,dt,tend,x,x_dot,sig,sig_hat,w,w_hat,k,P1,d,dr,dr_h,v,vr,vr_h,Ld,m,g,Wm,sigC,sigC_dot] = startP( q )
if q==1
m=0.429;

g=9.8;

I = [4.238*10^-3 0 0;0 4.986*10^-3 0;0 0 8.804*10^-3];%[0.004 0 0;0 0.004 0;0 0 0.008];%% inertia
%I=[22.3*10-4 0 0; 0 29.8*10^-4 0; 0 0 48*10^-4];
initialPosition = [3;2;10];

initialVelocity = [0;0;0];

x=initialPosition;

x_dot=initialVelocity;
%q=[cos(180*pi/360);(cos(90*pi/180))*sin(-180*pi/360);(sin(90*pi/180))*sin(-180*pi/360);0];

%q=[cos(180*pi/360);(1/sqrt(3))*sin(180*pi/360);(1/sqrt(3))*sin(180*pi/360);(1/sqrt(3))*sin(180*pi/360)];

%q=[cos(180*pi/360);(1/sqrt(3))*sin(0*pi/360);(1/sqrt(3))*sin(0*pi/360);sin(180*pi/360)];
vec=[0,1,0];
vec=vec/norm(vec);
phi=0;
q=[cos(phi*pi/360);vec(1)*sin(phi*pi/360);vec(2)*sin(phi*pi/360);vec(3)*sin(phi*pi/360)];

sig=[q(2)/(1+q(1));q(3)/(1+q(1));q(4)/(1+q(1))];
%sig=[0 0.2+1/(1+sqrt(2)) 0]';% initial guess (estimated and actual ) actual=real


sig_hat=sig;%[0 0.2+1/(1+sqrt(2)) 0]';% this is not equal to sigBN_real

w=[ 0 360 0]'*(0/180);% angular velocity  initial guess (estimated and actual)

w_hat=[ 0 360 0]'*(0/180);

dt=0.01;% one step length

k=5;% gain

P1=1*eye(6);% STATE COVARIANCE MATRIXS

d=[0.1 0.32 0.2]';

dr=[0.1 0.32 0.2]';

dr_h=[0.1 0.32 0.2]';

v=[0.1 ;0.2; 0.3];

vr=[0.1 ;0.2; 0.3];

vr_h=[0.1 ;0.2; 0.3];


Ld=3*eye(3,3);

tend=170.5;

Wm=[0.1;0.1;0];

sigC=[0;0;0];

sigC_dot=[0;0;0];

end
end
