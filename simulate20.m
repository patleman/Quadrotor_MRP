clc
clear 

[ I,dt,tend,x,x_dot,sig,sig_hat,w,w_hat,k,P,d,dr,dr_h,v,vr,vr_h,Ld,m,g,Wm,sigC,sigC_dot] = startP(1);
ten=(tend/dt)+1;
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



Ur_h=[0;0;0];
fex=1;
firsttime=0;

maxi=[0;0;0];
maxF=0;
DA=eye(1,ten);
Va=eye(3,ten);
%load('commandNet.mat')
fs = 16e3;
%classificationRate = 20;
%audioIn = audioDeviceReader('SampleRate',fs, ...
%'SamplesPerFrame',floor(fs/classificationRate));
%frameLength = floor(frameDuration*fs);
%hopLength = floor(hopDuration*fs);
%waveBuffer = zeros([fs,1]);

%labels = trainedNet.Layers(end).Classes;
%YBuffer(1:classificationRate/2) = categorical("background");
%probBuffer = zeros([numel(labels),classificationRate/2]);
%h = figure('Units','normalized','Position',[0.2 0.1 0.6 0.8]);

vvv = 1;
ttt=1;
for t=0:dt:tend 
    b=[0;0;1];
    a=(mrpTOdcm(sig_hat)')*b;%if degAngle>=39 && firsttime==0 &&  fex==0 % 
    angle = atan2(norm(cross(b,a)), dot(b,a));
    degAngle=angle*(180/pi);
    if degAngle<30 && firsttime==0
        firsttime=1;
    end
    if degAngle>30 && firsttime==0
        fex=1;
        t
    else
        fex=0;
    end
    
     dreal=disturbanceG(t,1);
     
%while ishandle(h)  
 ttt=0;
 vvv=3;
% while ttt==1
%  % Extract audio samples from the audio device and add the samples to
%     % the buffer.
%     x12 = audioIn();
%     waveBuffer(1:end-numel(x12)) = waveBuffer(numel(x12)+1:end);
%     waveBuffer(end-numel(x12)+1:end) = x12;
% 
%     % Compute the spectrogram of the latest audio samples.
%     spec = melSpectrogram(waveBuffer,fs, ...
%         'WindowLength',frameLength, ...
%         'OverlapLength',frameLength - hopLength, ...
%         'FFTLength',512, ...
%         'NumBands',numBands, ...
%         'FrequencyRange',[50,7000]);
%     spec = log10(spec + epsil);
% 
%     % Classify the current spectrogram, save the label to the label buffer,
%     % and save the predicted probabilities to the probability buffer.
%     [YPredicted,probs] = classify(trainedNet,spec,'ExecutionEnvironment','cpu');
%     YBuffer(1:end-1)= YBuffer(2:end);
%     YBuffer(end) = YPredicted;
%     probBuffer(:,1:end-1) = probBuffer(:,2:end);
%     probBuffer(:,end) = probs';
% 
%     % Plot the current waveform and spectrogram.
%     subplot(2,1,1);
%     plot(waveBuffer)
%     axis tight
%     ylim([-0.2,0.2])
% 
%     subplot(2,1,2)
%     pcolor(spec)
%     caxis([specMin+2 specMax])
%     shading flat
% 
%     % Now do the actual command detection by performing a very simple
%     % thresholding operation. Declare a detection and display it in the
%     % figure title if all of the following hold:
%     % 1) The most common label is not |background|.
%     % 2) At least |countThreshold| of the latest frame labels agree.
%     % 3) The maximum predicted probability of the predicted label is at
%     % least |probThreshold|. Otherwise, do not declare a detection.
%     [YMode,count] = mode(YBuffer);
%     countThreshold = ceil(classificationRate*0.2);
%     ar_ray=probBuffer(labels == YMode,:);
%     maxProb = max(ar_ray);
%     probThreshold = 0.7;
%     subplot(2,1,1);
%     if YMode == "background" || count<countThreshold  || maxProb < probThreshold
%         title(" ")
%     else
%         title(string(YMode),'FontSize',20)
%     end
% 
%     drawnow
% 
%      if YMode == "yes"         %  circle
%          vvv = 2;
%          ttt=0;
%        %  break;
%      elseif YMode=="up"          % 8-shape
%          vvv =1 ;
%          ttt=0;
%        %  break;
%      elseif YMode=="go"              % spiral
%          vvv =3 ;
%          ttt=0;
%         % break;
%      end
% end
%end     
     [xr,xr_dot,xrd_dot,psid]=refTraj(x,t,vvv);
     [v,d]=NDOt(v,d,m,U,dt,xr_dot,x_dot,xrd_dot);
     if fex==1 %norm(sig_hat)^2>setcheck
         U=[0;0;0];  
     else
     [U,error,maxi]=controlT(x,x_dot,xr,xr_dot,xrd_dot,d,m,maxi);
   
     end
 
        if degAngle>90 && firsttime==0
          %    U=.2*(U/norm(U)); 
             % t
              x
              w
              sig_hat
              norm(sig_hat)^2
             
        end
       if fex==1
           sigC=[0;0;0];
           sigC_dot=[0;0;0];
           wC_dot=[0;0;0];
           wC=[0;0;0];
       else
  
    [Fbody_h,sigD,sigC,sigC_dot,wC_dot,wC] = attTraj1(m,U,g,psid,sigC,sigC_dot,dt);% attitude trajectory 
       end

    dreal_r=disturbanceG(t,2);
    
   % dreal_r=[0;0;0];
  
    for n=t:dt/10:t+dt-(dt/10)    %%%inner loop
        if fex==1 
      [Ur_h,esig_sum_h,desig_sum_h,Fbody_h]=controlR2(sig_hat,w_hat,sigC,wC_dot,wC,I,dr_h,esig_sum_h,desig_sum_h);
              if Fbody_h>maxF
                %  t
                 maxF=Fbody_h
              end
        else
      [Ur_h,esig_sum_h,desig_sum_h]=controlR(sig_hat,w_hat,sigC,wC_dot,wC,I,dr_h,esig_sum_h,desig_sum_h);
        end
       
   [Ur,esig_sum,desig_sum]=controlR(sig,w,sigC,wC_dot,wC,I,dr,esig_sum,desig_sum);
   [vr,dr]=NDOr(vr,dr,I,Ur,dt/10,w);
    
    [vr_h,dr_h]=NDOr(vr_h,dr_h,I,Ur_h,dt/10,w_hat);  
   %  dr=[0;0;0];
    % dr_h=[0;0;0];
   % [f1,f2,f3,f4,ff]=ThrustTorqueToMf(Fbody_h,Ur_h);
  %  Ur_h=[ff(2);ff(3);ff(4)];
   %   Fbody_h=ff(1);
    [sig,w]=RotationalDynamics(Ur,w,sig,dreal_r,I,dt/10,Fbody_h);
    
    if norm(sig)>1
    sig=switchMRP(sig);
    end
    
    [sig_hat,w_hat]=RotationalDynamics(Ur_h,w_hat,sig_hat,dreal_r,I,dt/10,Fbody_h);
    
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
   % sig_hat=sig;
    
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
[f1,f2,f3,f4,ff]=ThrustTorqueToMf(Fbody_h,Ur_h);
Mf=[f1;f2;f3;f4];

MFlist(:,i)=Mf;
dcm0=mrpTOdcm(sig);
%[y0,p0,r0]=dcm2angle(dcm0);
siglist(:,i)= sig_hat;%[r0;p0;y0];%*(180/pi);%sigD;rpy0*(180/pi);sig_hat;

dcm1=mrpTOdcm(sigC);
%[y1,p1,r1]=dcm2angle(dcm1);
siglist1(:,i)= sigC;%[r1;p1;y1];%*(180/pi);%sig;rpy1*(180/pi);sigC;

x_dotlist(:,i)=x_dot;

disturbanceT(:,i)=d;

disturbanceR(:,i)=dr;
i=i+1;

 % t  
end

figure(30)
plot(DA)
plotFigures(xlist,xrlist,wlist,errorlist,siglist,siglist1)
plotFiguresD(disturbanceR,disturbanceT,Thrust,Torque,MFlist)

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

data.x=xlist;
data.sig=siglist;
data.input=MFlist;
data.angvel=wlist;
data.t=1:ten;
visualize3d(data)