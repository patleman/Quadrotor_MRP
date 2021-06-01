function [sig,w,P]=EKF_MRP(sig,w,P,sigM,wM,I,dt)

%%%% process covariance noise and measurement covariance noise
%Q=10*10^-9*eye(6);
Q=diag([0,0,0,1,1,1]);
R=4*10^-4*eye(6);


%%% F matrix computation/ jacobian of f(x)
F11=(1/2)*(sig*(w')-w*(sig')-skew(w)+(sig')*w*I);
sig_dot=(1/4)*((1-sig'*sig)*eye(3)+2*skew(sig)+2*(sig)*sig');
F12=sig_dot;
F21=zeros(3,3);
F22=[0 -w(3)*I(2)+I(3)*w(3)   -w(2)*I(2)+I(3)*w(2);
     I(1)*w(3)-I(3)*w(3) 0 I(1)*w(1)-I(3)*w(1);
    I(2)*w(2)-I(1)*w(2) -w(1)*I(1)+w(1)*I(2) 0];
F22=I\F22;
F=[F11 F12;F21 F22];

%%%% G matrix computation / jacobian of g/ coeffecient of wgn
G=[zeros(3,3) zeros(3,3);zeros(3,3) inv(I)];


%%% state covariance prediction
P_dot=F*P+P*(F')+G*Q*(G');
P=P+P_dot*dt;

%%% Measurement matrix
H=[eye(3) zeros(3,3);zeros(3,3) eye(3)];

%%%  KALMAN GAIN
Kstep=H*P*(H')+R;

K=P*(H')/Kstep;


%%% measurement-prediction step
Yk=sigM-sig;
if norm(sigM)>1/3
    Yk_d=switchMRP(sigM)-sig;
    if norm(Yk_d)<norm(Yk)
        Yk=Yk_d;
    end
end
Wk=wM-w;
error=[Yk;Wk];


%%% correction step/updtae step
update=K*error;
sig=sig+dt*update(1:3);
 if norm(sig)>1
    sig=switchMRP(sig);
    P=covarianceChange(P,sig);
 end
w=w+dt*update(4:6);


%%% update P
P=(eye(6)-K*H)*P*((eye(6)-K*H)')+K*R*(K');
end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end