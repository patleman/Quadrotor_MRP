function P=covarianceChange(p,sig)
p11=p(1:3,1:3);
p12=p(1:3,4:6);
p21=p(4:6,1:3);
p22=p(4:6,4:6);
n=norm(sig)^2;
S=2*(1/(n^2))*((sig)*sig')-(1/n)*eye(3);
P=[S*p11*(S') S*p12; p21*(S') p22];

end