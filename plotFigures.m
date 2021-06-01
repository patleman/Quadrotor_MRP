function plotFigures(xlist,xrlist,wlist,errorlist,siglist,siglist1)
 % 
 %s=load('withoutEKF.mat');
 %siglistWFsig=s.siglistCopy;
 
 figure(2)   % 3D TRAJECTORY
 plot3(xlist(1,:),xlist(2,:),xlist(3,:),'-b','LineWidth',1);
 hold on
 plot3(xrlist(1,:),xrlist(2,:),xrlist(3,:),'--r','LineWidth',1);
 
 hold on
 ylabel('\fontsize{16} Y(m)')
 xlabel('\fontsize{16} X(m)')
 zlabel('\fontsize{16} Z(m)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} 3D TRAJECTORY')
 
 
 
 
 figure(3)
 plot( xlist(1,:),'-b','LineWidth',1)
 hold on 
 plot( xrlist(1,:),'--r','LineWidth',1)
 hold on
 ylabel('\fontsize{16} X(m)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} X-Plot')
 
 figure(4)
 plot( xlist(2,:),'-b','LineWidth',1)
 hold on 
 plot( xrlist(2,:),'--r','LineWidth',1)
 hold on
 ylabel('\fontsize{16} Y(m)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} Y-Plot')
 
 
 
 
 figure(5)
 plot( xlist(3,:),'-b','LineWidth',1)
 hold on 
 plot( xrlist(3,:),'--r','LineWidth',1)
 hold on
 ylabel('\fontsize{16} Z(m)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} Z-Plot')
 
 figure(6)

 plot( siglist(1,:),'-b','LineWidth',1)
 hold on 
 plot( siglist1(1,:),'--r','LineWidth',1)
 hold on
 ylabel('\fontsize{16} Sigma(1)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} MRP(1)')
 
 
 figure(7)
 
 plot( siglist(2,:),'-b')
 hold on 
 plot( siglist1(2,:),'--r')
 hold on
 ylabel('\fontsize{16} Sigma(2)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} MRP(2)')
 
 figure(8)
 plot( siglist(3,:))
 hold on 
 plot( siglist1(3,:))
 plot( siglist(3,:),'-b')
 hold on 
 plot( siglist1(3,:),'--r')
 hold on
 ylabel('\fontsize{16} Sigma(3)')
 xlabel('\fontsize{16} Time(msec)')
 legend('\fontsize{16} Actual','\fontsize{16} Desired');
 title('\fontsize{16} MRP(3)')
 
 
 figure(9)
 plot(errorlist(1,:))
 hold on 
 plot(errorlist(2,:))
 hold on 
 plot(errorlist(3,:))
 
 
 figure(10)
 plot(wlist(1,:))
 hold on
 plot(wlist(2,:))
 hold on
 plot(wlist(3,:))

end