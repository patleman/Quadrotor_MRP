clc
clear 
ten=12001;
[ I,dt,tend,x,x_dot,sig,sig_hat,w,w_hat,k,P,d,dr,dr_h,v,vr,vr_h,Ld,m,g,Wm,sigC,sigC_dot] = startP(1);
siglist=eye(3,ten);

siglist1=eye(3,ten);

wlist=eye(3,ten);

xlist=eye(3,ten);

x_dotlist=eye(3,ten);

errorlist=eye(3,ten);

disturbanceT=eye(3,ten);

disturbanceR=eye(3,ten);

Thrust=eye(3,ten);

Torque=eye(3,ten);

xrlist=eye(3,ten);% reference trajectory
MFlist=eye(4,ten);% motor forces

i=1;
 esig_sum=[0;0;0];
   desig_sum=[0;0;0];
   esig_sum_h=[0;0;0];
   desig_sum_h=[0;0;0];
   U=[0.2;0.1;0.2];
   j=1;
   fa=zeros(1,11);
   tlist=-1*ones(1,11);% time when it hits the ground
   zlist=zeros(1,11);

min=x(3);
Ur_h=[0;0;0];
fex=1;
firsttime=0;
chan=0;
setcheck=0.1718;
setcheck2=0.0296;
control=1;
max=[0;0;0];
maxF=0;
z=[0;0;0];
DA=eye(1,ten);
Va=eye(3,ten);
for t=0:dt:tend 
    b=[0;0;1];
    a=(mrpTOdcm(sig_hat)')*b;%if degAngle>=39 && firsttime==0 &&  fex==0 % 
    angle = atan2(norm(cross(b,a)), dot(b,a));
    degAngle=angle*(180/pi);
  %  if degAngle<30 && firsttime==0
   %     firsttime=1;
   % end
   % if degAngle>30 && firsttime==0
     %   fex=1;
    %    t
   % else
        %fex=0;
   % end
     dreal=disturbanceG(t,1);
       
     [xr,xr_dot,xrd_dot,psid]=refTraj(x,t,3);
     [v,d]=NDOt(v,d,m,U,dt,xr_dot,x_dot,xrd_dot);
     %if fex==1 %norm(sig_hat)^2>setcheck
     %    U=[0;0;0];  
    % else
     [U,error,max]=controlT(x,x_dot,xr,xr_dot,xrd_dot,d,m,max);
   
    % end
 
       % if degAngle>90 && firsttime==0
          %    U=.2*(U/norm(U)); 
             % t
            %  x
           %   w
            %  sig_hat
            %  norm(sig_hat)^2
             
       % end
      % if fex==1
        %   sigC=[0;0;0];
        %   sigC_dot=[0;0;0];
        %   wC_dot=[0;0;0];
         %  wC=[0;0;0];
    %   else
  
    [Fbody_h,sigD,sigC,sigC_dot,wC_dot,wC] = attTraj1(m,U,g,psid,sigC,sigC_dot,dt);% attitude trajectory 
     %  end

    dreal_r=disturbanceG(t,2);
    
   % dreal_r=[0;0;0];
  
    for n=t:dt/10:t+dt-(dt/10)    %%%inner loop
      %  if fex==1 
     % [Ur_h,esig_sum_h,desig_sum_h,Fbody_h]=controlR2(sig_hat,w_hat,sigC,wC_dot,wC,I,dr_h,esig_sum_h,desig_sum_h);
         %     if Fbody_h>maxF
                %  t
           %      maxF=Fbody_h
          %    end
      %  else
      [Ur_h,esig_sum_h,desig_sum_h]=controlR(sig_hat,w_hat,sigC,wC_dot,wC,I,dr_h,esig_sum_h,desig_sum_h);
       % end
       
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
         disp('hello')
    sig_hat=switchMRP(sig_hat);
    P=covarianceChange(P,sig_hat);
    end
  
    w_hat=w_hat+sqrt(10*10^-9)*randn(3,1);% adding process noise
   
    
    %%% KALMAN FILTER IMPLEMENTATION
    %%%% measurements
    
    sigM=addMRP(sig,sqrt(4*10^-7)*randn(3,1));
    
    wM=w+sqrt(4*10^-7)*randn(3,1);
     
    %%%%% KALMAN FILTER
    
   [sig_hat,w_hat,P]=EKF_MRP(sig_hat,w_hat,P,sigM,wM,I,dt/10);
 
    U=orientedU(Fbody_h,sig_hat,m);% this expression is in inertial/world frame
    [x,x_dot]=TranslationalDynamics(U,x_dot,x,dreal,m,dt/10);
    end
    fex=0;

xlist(:,i)=x;

xrlist(:,i)=xr;

DA(i)=degAngle;
Va(:,i)=a;
wlist(:,i)= w_hat;%[P(1,1),P(2,2),P(3,3)];%

errorlist(:,i)=norm(addMRP(sig_hat,-sigC));


Thrust(:,i)=Fbody_h;
Torque(:,i)=Ur_h;
[f1,f2,f3,f4]=ThrustTorqueToMf(Fbody_h,Ur_h);
Mf=[f1;f2;f3;f4];

MFlist(:,i)=Mf;
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

    
end



figure(30)
plot(DA)
plotFigures(xlist,xrlist,wlist,errorlist,siglist,siglist1)
plotFiguresD(disturbanceR,disturbanceT,Thrust,Torque,MFlist)
figure(100)
N = 12001 ;
%%coordinate positions30;%30;%
X = Va(1,:) ;
Y = Va(2,:) ;
Z = Va(3,:) ;
%%Velocity components
u = x_dotlist(1,:) ;
size(u)
v = x_dotlist(2,:) ;
w = x_dotlist(3,:) ;
%for i = 1:N
  %  quiver3(X(i),Y(i),Z(i),u(i),v(i),w(i)) ;
 %   pause(0.01) ;
%end
a1=152+176+176+172+196+168+208;
a1=a1-20-24+32+32+64;
b1=20+24+22+22+24+24+26;
b1=b1+8;
data.x=xlist;
data.sig=siglist;
data.input=MFlist;
data.angvel=wlist;
data.t=1:ten;
visualize3d(data)