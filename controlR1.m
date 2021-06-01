function [Ur,esig_sum,desig_sum,Fbody]=controlR1(sig,w,sigC,wC_dot,wC,I,dr,esig_sum,desig_sum)
k=2;% gain
P=5*eye(3);% gain matrix
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
Ur=-k*sig_error+I*(wC_dot-cross(w,wC))+wBN_tilde_real*I*w-dr;%-P*w_error
d=0.1785;% distance betweeen centre and line connecting two adjacent motors
%c=0.08;
c=(2.423*10^-7)/(8.050*10^-6);
A=[d d -d -d;d -d -d d;c -c c -c];
b=[Ur(1); Ur(2) ;Ur(3)];

M = [A b];
m=[M(3,:);M(2,:);M(1,:)];
[R,p] = rref(m);
array=[R(1,5);R(2,5);R(3,5)];
max_f=abs(R(1,5));
for t=2:3
    if abs(array(t))>max_f
        max_f = abs(array(t));  
    end
end
%abs(R(3,5))
f4=max_f
f1=-f4*R(1,4)+R(1,5)
f2=-f4*R(2,4)+R(2,5)
f3=-f4*R(3,4)+R(3,5)
Fbody=f1+f2+f3+f4
desig_sum=w_error;

end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end