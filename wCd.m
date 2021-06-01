function [wC_dot]=wCd(sigC,sigC_dot,sigCd_dot)
s1=sigC;
s2=sigC_dot;
s3=sigCd_dot;
Gsig=(1/4)*((1-(s1')*s1)*eye(3)+2*skew(s1)+2*s1*(s1'));

Gsig_dot_1=(8/((1+(s1')*s1)^2))*(s2*(s1')+s1*(s2')-(s2')*(s1)*eye(3)-skew(s2));
Gsig_dot_2=((16*(s1')*s2)/((1+(s1')*s1)^3))*((1-(s1')*s1)*eye(3)-2*skew(s1)+2*s1*(s1'));

Gsig_dot = Gsig_dot_1-Gsig_dot_2 ;

G_dot_inverse=-inv(Gsig)*Gsig_dot*inv(Gsig);

wC_dot=G_dot_inverse*s2+inv(Gsig)\s3;
end
function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];

end