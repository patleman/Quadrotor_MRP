function [xr,xr_dot,xrd_dot,psid]=refTraj(xrp,t,n )
% circle trajectory
if n==1

xr=[4*cos(0.1*t);4*sin(0.1*t);10];
xr_dot=[-.5*0.1*(sin(0.1*t));.5*0.1*cos(0.1*t);0];
xrd_dot=[-0.5*(0.1)^2*cos(0.1*t);-0.5*(0.1)^2*sin(0.1*t);0];
%xr=[3;2;10];
%xr_dot=[0;0;0];
%xrd_dot=[0;0;0];
psid=0.6;
end
% 8-shape curve
if n==2
   
xr=[5*(1-cos((pi/18)*t));5*sin((pi/9)*t);10];%[10;10;10];%
xr_dot=[5*(pi/18)*(sin((pi/18)*t));5*(pi/9)*cos((pi/9)*t);0];%[0;0;0];%
xrd_dot=[5*(pi/18)^2*(cos((pi/18)*t));-5*(pi/9)^2*sin((pi/9)*t);0];%[0;0;0];%
psid=0;
end
% spiral-trajectory
if n==3
 xr=[5*sin(2*t/10);5*cos(2*t/10);0.5*t+10];
 xr_dot=[1*cos(2*t/10) ;-1*sin(2*t/10);0.5 ];
 xrd_dot=[-(1/5)*sin(2*t/10);-(1/5)*cos(2*t/10);0];
 psid=0.6;
end
% up 4
if n==4
 xr=xrp+[0; 0 ;1];
 xr_dot=[0;0 ;0];
 xrd_dot=[0;0;0];
 psid=0.6;
end
% right 5
if n==5
 xr=xrp+[0; 1 ;0];
 xr_dot=[0;0 ;0];
 xrd_dot=[0;0;0];
 psid=0.6;
end
% left 6
if n==6
 xr=xrp+[1; 0 ;0];
 xr_dot=[0;0 ;0];
 xrd_dot=[0;0;0];
 psid=0.6;
end
end