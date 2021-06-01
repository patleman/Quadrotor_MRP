function [Ur,esig_sum,desig_sum,Fbody]=controlR2(sig,w,sigC,wC_dot,wC,I,dr,esig_sum,desig_sum)

b=[0;0;1];
a=(mrpTOdcm(sig)')*b;%if degAngle>=39 && firsttime==0 &&  fex==0 % 
degAngle =(180/pi)* atan2(norm(cross(b,a)), dot(b,a));
d=0.1785;
c=(2.423*10^-7)/(8.050*10^-6);
A=[d d -d -d;d -d -d d;c -c c -c];
Motor_cordsB=[-d,d,d,-d;
               d,d,-d,-d;
                0,0,0,0];
Motor_cordsI=((mrpTOdcm(sig))')*Motor_cordsB
minZ=Motor_cordsI(3,1);% refereing to motor 1
m1=1;
%m2=0;
%mat=zeros(4,4);
for t=2:4
if Motor_cordsI(3,t)<=minZ
   % if  Motor_cordsI(3,t)==minZ
     %   mat(m1,t)=1;
 %   else
       minZ=Motor_cordsI(3,t); 
       m1=t;
    %   if m1>=2
     %   m2=m1-1;
      %  else
       % m2=m1+1;
       %end
    %end
end
end
%if minZ==0 
   
   % m1=1;
    
%end
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
%elseif Motor_cordsI(3,mf)==Motor_cordsI(3,mb)
     %  m2=0; 
else
    if Motor_cordsI(3,mf)==Motor_cordsI(3,mb)
        m2=0;
    else
    m2=mb;
    end
end

f_v=zeros(4,1);
f_v(m1)=2+2.025*(degAngle/180);
%if m2~=0
if m2~=0
f_v(m2)=2+2.025*(degAngle/180);
end
%end
f_v
Ur=A*f_v;
Fbody=f_v(1)+f_v(2)+f_v(3)+f_v(4);

end
