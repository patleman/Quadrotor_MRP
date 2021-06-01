function dreal=disturbanceG(t,x)
if x==1
%dreal=[.8*sin(0.1*t);1*sin(0.1*t);.5*sin(0.1*t)];
dreal=[0;0;0];
else
 %dreal=[.02*sin(0.1*t);.05*sin(0.1*t);0.05];%.04*sin(0.1*t)
 dreal=[0;0;0];%.04*sin(0.1*t)
end
end