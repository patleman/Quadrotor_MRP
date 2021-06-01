function [U,error,max]=controlT( x,x_dot,xr,xr_dot,xrd_dot,d,m,max)
l1=2.5;
l2=2.5;
%errorDOT=x_dot-xr_dot;

k1=4;
k2=3;

errorDOT=x_dot-xr_dot;
error=x-xr;
%U=xrd_dot+K2*(errorDOT)+K1*(error)-d;
U=-l1*tanh(k1*error)-l2*tanh(k2*errorDOT)+xrd_dot+[0;0;9.8]-(d/m);%+k2*(errorDOT)
%if U(3)<0
   % norm(U)
   % U(3)
   if norm(U)>max
      max=0.429*norm(U);
   end
    c=U(3)/norm(U);
    if c==-1
        U(3)
    end
%end
%error
%U=-k1*error-k2*errorDOT+xrd_dot+[0;0;9.8]-(d/m);
%if U(3)<-9.8
 %  U(3)=0
%end

end