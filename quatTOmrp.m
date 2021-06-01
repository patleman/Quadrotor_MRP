function mrp=quatTOmrp(q)
% quaternion [q0 q1 q2 q3 q4]
d=1+q(1);
a=q(2)/d;
b=q(3)/d;
c=q(4)/d;
mrp=[a;b;c];
end