function plotFiguresD(R,T,t,t1,motor)
 % 
 %s=load('withoutEKF.mat');
 %siglistWFsig=s.siglistCopy;
 figure(11)
 plot( R(1,:),'-b','LineWidth',1.5)
 hold on 
 plot(R(2,:),'--r','LineWidth',1.5)
 hold on
 plot( R(3,:),'g','LineWidth',1.5)
 hold on
 ylabel('\fontsize{16} Disturbance(Nm)')
 xlabel('\fontsize{16}time (msec)')
 legend('\fontsize{16} About X','\fontsize{16}About Y','\fontsize{16}About Z');
 title('\fontsize{16} Disturbances about X,Y and Z')
 
 figure(12)
 plot( T(1,:),'-b','LineWidth',1.5)
 hold on 
 plot(T(2,:),'--r','LineWidth',1.5)
 hold on
 plot( T(3,:),'g','LineWidth',1.5)
 hold on
 ylabel('\fontsize{16} Disturbance(N)')
 xlabel('\fontsize{16}time (msec)')
 legend('\fontsize{16} Along X','\fontsize{16}Along Y','\fontsize{16}AlongZ');
 title('\fontsize{16} Disturbances in X,Y and Z')
 
 figure(13)

 plot( t(3,:),'g')
 hold on
 ylabel('\fontsize{16} Force(N)')
 xlabel('\fontsize{16}time (msec)')
 legend('\fontsize{16}Body frame Z');
 title('\fontsize{16} Thrust')
 
 figure(14)
 plot( t1(1,:),'-b')
 hold on 
 plot(t1(2,:),'--r')
 hold on
 plot( t1(3,:),'g')
 hold on
 ylabel('\fontsize{16} Torque(Nm)')
 xlabel('\fontsize{16}time (msec)')
 legend('\fontsize{16}x','\fontsize{16}  y','\fontsize{16}  Z');
 title('\fontsize{16} Torque')
 
 figure(15)
 plot( motor(1,:),'-b')
 hold on 
 plot(motor(2,:),'--r')
 hold on
 plot( motor(3,:),'g')
 hold on
 plot( motor(4,:),'--g')
 hold on
 ylabel('\fontsize{16} Thrust(N)')
 xlabel('\fontsize{16}time (msec)')
 legend('\fontsize{16}f1','\fontsize{16} f2','\fontsize{16} f3','\fontsize{16} f4');
 title('\fontsize{16} MOTOR f ')
end

