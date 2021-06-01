function [sig,w]=RotationalDynamics(Ur,w,sig,dreal_r,I,dt,Fbody_h)

%d=0.1785;
%c=(2.423*10^-7)/(8.050*10^-6);
%c=0.01;
%mat=[1 1 1;d d -d;d -d -d];
%vec=[Fbody_h;Ur(1);Ur(2)];
%Fs=mat\vec;
%F1=Fs(1);
%F2=Fs(2);
%F3=Fs(3);

%if F1<0 || F2<0 ||F3<0
 %   disp('minus')
   % F1
  %  F2
 %   F3
%end
%Ur(3)=c*(F1-F2+F3);
wBN_tilde_real = skew(w) ;
sig_tilde = skew(sig);
w_dot=I\(dreal_r+Ur-wBN_tilde_real*I*w);
sig_dot=(1/4)*((1-sig'*sig)*eye(3)+2*sig_tilde+2*(sig)*sig')*w;

w= w + dt*w_dot;
sig= sig + dt*sig_dot;

end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];
end