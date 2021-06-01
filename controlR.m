function [Ur,esig_sum,desig_sum]=controlR(sig,w,sigC,wC_dot,wC,I,dr,esig_sum,desig_sum)
k=[5 0 0;0 5 0;0 0 5];%15;% gain
P=[1 0 0; 0 1 0; 0 0 1];% gain matrix
Ki=0.003*eye(3);

%l1=3;
%l2=7;
%k3=0.1;
%k4=0.2;
%l3=1;
sig_error=addMRP(sig,-sigC);

if norm(sig_error)>1
sig_error=switchMRP(sig_error);
end


mrpdcm_real=mrpTOdcm(sig_error);

w_error = w-mrpdcm_real*wC;
esig_sum=esig_sum+sig_error;
wBN_tilde_real= skew(w) ;


%Ur=-k*tanh(l1*sig_error)-P1*tanh(l2*w_error)+I*(wC_dot-cross(w,wC))+wBN_tilde_real*I*w-dr;
Ur=-k*sig_error-(P+P*Ki*I)*w_error+I*(wC_dot-cross((mrpTOdcm(sig))'*w,wC))+wBN_tilde_real*I*w-dr-(k*P*Ki)*esig_sum+(P*Ki*I)*desig_sum;
%Ur=-P*w_error;
desig_sum=desig_sum+w_error;
%Ur(3)=0;
end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end