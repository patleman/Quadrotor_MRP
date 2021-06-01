function [f1,f2,f3,f4,ff]=ThrustTorqueToMf(Th,Tr)

l1=0.1785;% distance betweeen centre and line connecting two adjacent motors
%l3=0.1285;
%c=0.08;

d=2.423*10^-7;

b=8.050*10^-6;

c=d/b;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% assuming to be trirotor
%U1=Th;
%U2=Tr(1);
%U3=Tr(2);

%f2=(U1/2)-(U2/2*l1)-((l1-U3)/2*b*(l1+l3));

%f3=(l1/2*(l1+l3))-(U3/2*(l1+l3));


%f1=(U1/2)+(U2/(2*l1))-((l1-U3)/2*(l1+l3));

%f4=0;






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%c=0.01;
%mat=[1 1 1 1;
 %   l1 l1 -l1 -l1;
  %  l1 -l1 -l1 l1;
   % -c c -c c];
mat=[1 1 1;
   l1 l1 -l1;
    l1 -l1 -l1;
   -c c -c];
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% try removing yaw moment from the picture
%mat=[1 1 1;
   %  d d -d;
 %   d -d -d];
 
%vec=[Th;Tr(1);Tr(2)];
%vecmf=mat\vec;
%%%%%%%%%%%%%%%%%%%%%%%%%
vec=[Th;Tr(1);Tr(2);Tr(3)];
vecmf=mat\vec;
f1=vecmf(1);
f2=vecmf(2);
f3=vecmf(3);
f4=0;
%f1=2.044;
%f2=0.11;
%f3=2.044;
%f4=0;
%f4=vecmf(4);
if f1<0 || f2<0 || f3<0 %|| f4<0
 if  f1<0    
   f1
 end
 if  f2<0    
   f2=0
 
 end
 if  f3<0    
   f3
 end
 if  f4<0    
   f4
 end
 
   
end

dd=[f1;f2;f3];
ff=mat*dd;
%if vec(4) == ff(4)
 %   disp("nice")
%end
end