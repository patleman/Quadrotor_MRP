clc
clear 

[ I,dt,tend,x,x_dot,sig,sig_hat,w,w_hat,k,P,d,dr,dr_h,v,vr,vr_h,Ld,m,g,Wm,sigC,sigC_dot] = startP(1);
%P8=1*eye(9);
P8=diag([1,1,1,1,1,1,0.5,0.5,0.5]);
siglist=eye(3,tend);


Roty=0;


siglist1=eye(3,tend);

wlist=eye(3,tend);

xlist=eye(3,tend);

x_dotlist=eye(3,tend);

errorlist=eye(3,tend);

disturbanceT=eye(3,tend);

disturbanceR=eye(3,tend);

Thrust=eye(3,tend);

Torque=eye(3,tend);

xrlist=eye(3,tend);

i=1;
 esig_sum=[0;0;0];
   desig_sum=[0;0;0];
   esig_sum_h=[0;0;0];
   desig_sum_h=[0;0;0];
   U=[0;0;0];
   wC=[0;0;0];
firstexi=0;

for t=0:dt:tend
     
    
   % dreal=disturbanceG(t,1);
     dreal=[0;0;0];
 
    [xr,xr_dot,xrd_dot,psid]=refTraj(t,1);
    [v,d]=NDOt(v,d,m,U,dt,xr_dot,x_dot,xrd_dot);
     if norm(sig_hat)^2 >0.1716 %  beyond90 degres
        zeta=.707;
        wn=12;
        sigC=sigC+dt*sigC_dot;
        sigCd_dot=-2*zeta*wn*sigC_dot-(wn^2)*(sigC-[0;0;0]);

         sigC_dot=sigC_dot + dt*sigCd_dot;

          wC=sdot(sigC)\sigC_dot;

          wC_dot=wCd(sigC,sigC_dot,sigCd_dot); 
          Roty=1;
     else
         if firstexi==0
             sigC=[0;0;0];
           sigC_dot=[0;0;0];
         end
     [U,error]=controlT(x,x_dot,xr,xr_dot,xrd_dot,d,m);
    
   
     [Fbody,sigD,sigC,sigC_dot,wC_dot,wC] = attTraj1(m,U,g,psid,sigC,sigC_dot,dt);% attitude trajectory 
     end
   %  dreal_r=disturbanceG(t,2);
    
     dreal_r=[0;0;0];
  
    for n=t:dt/10:t+dt-(dt/10)    %%%inner loop
        
    [Ur_h,esig_sum_h,desig_sum_h]=controlR(sig_hat,w_hat,sigC,wC_dot,wC,I,dr_h,esig_sum_h,desig_sum_h);
    if Roty==1
        Fbody=Ur_h(2)/.125;%12.5 cm distance betweeen centre and line connecting two adjacent motors
    end
   
    [Ur,esig_sum,desig_sum]=controlR(sig,w,sigC,wC_dot,wC,I,dr,esig_sum,desig_sum);
    
   [vr,dr]=NDOr(vr,dr,I,Ur,dt/10,w);
    
    [vr_h,dr_h]=NDOr(vr_h,dr_h,I,Ur_h,dt/10,w_hat);
    
   %  dr=[0;0;0];
    % dr_h=[0;0;0];
    
    [sig,w]=RotationalDynamics(Ur,w,sig,dreal_r,I,dt/10);
    
    if norm(sig)>1
    sig=switchMRP(sig);
    end
    
    [sig_hat,w_hat]=RotationalDynamics(Ur_h,w_hat,sig_hat,dreal_r,I,dt/10);
    
    if norm(sig_hat)>1
    sig_hat=switchMRP(sig_hat);
    P=covarianceChange(P,sig_hat);
    end
  
   %sig_hat=sig_hat;
   % sig_hat=addMRP(sig_hat,[mvnrnd(0,10^-7);mvnrnd(0,10^-7);mvnrnd(0,10^-7)]);%%adding process noise
    
  %  w_hat=w_hat+[mvnrnd(0,10^-7);mvnrnd(0,10^-7);mvnrnd(0,10^-7)];% adding process noise
    w_hat=w_hat+sqrt(10*10^-9)*randn(3,1);% adding process noise
   
    
    %%% KALMAN FILTER IMPLEMENTATION
    %%%% measurements
    
    sigM=addMRP(sig,sqrt(4*10^-7)*randn(3,1));
    
    wM=w+sqrt(4*10^-7)*randn(3,1);
     %w_hat=wM;
    %%%%% KALMAN FILTER
    
   [sig_hat,w_hat,P]=EKF_MRP(sig_hat,w_hat,P,sigM,wM,I,dt/10);
  
    end
  % d=[0;0;0];
    U=orientedU(Fbody,sig_hat,m,d);% this expression is in inertial/world frame
    %[v,d]=NDOt(v,d,m,U,dt,xr_dot,x_dot,xrd_dot);
    [x,x_dot]=TranslationalDynamics(U,x_dot,x,dreal,m,dt);
    Roty=0;
if t==i%%%%%%%%%%%plotting work
    
xlist(:,i)=x;

xrlist(:,i)=xr;

wlist(:,i)= [P(1,1),P(2,2),P(3,3)];%wC-w;%

errorlist(:,i)=norm(addMRP(sig_hat,-sigC));


Thrust(:,i)=Fbody;
Torque(:,i)=Ur;
dcm0=mrpTOdcm(sig);
[y0,p0,r0]=dcm2angle(dcm0);
siglist(:,i)= sig;%[r0;p0;y0];%*(180/pi);%sigD;rpy0*(180/pi);sig_hat;

dcm1=mrpTOdcm(sigC);
[y1,p1,r1]=dcm2angle(dcm1);
siglist1(:,i)= sigC;%[r1;p1;y1];%*(180/pi);%sig;rpy1*(180/pi);sigC;

x_dotlist(:,i)=x_dot;

disturbanceT(:,i)=d;

disturbanceR(:,i)=dr;
i=i+1;
end          %plotting work end
    
end


plotFigures(xlist,xrlist,wlist,errorlist,siglist,siglist1)
plotFiguresD(disturbanceR,disturbanceT,Thrust,Torque)

function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];
end
function sd=sdot(x)
sd=(1/4)*((1-x'*x)*eye(3)+2*skew(x)+2*(x)*x');

end