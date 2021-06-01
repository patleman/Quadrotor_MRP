function amrp=addMRP(s1,s2)

%numerator = (1-(norm(s2)^2))*s1+(1-(norm(s1)^2))*s2-2*cross(s1,s2);
denominator = 1+(norm(s1)^2)*(norm(s2)^2)-2*(s1')*s2;
if denominator<0.002 & denominator>-0.002
    t
    s2=switchMRP(s2);
    numerator = (1-(norm(s2)^2))*s1+(1-(norm(s1)^2))*s2-2*cross(s1,s2);
    denominator = 1+(norm(s1)^2)*(norm(s2)^2)-2*(s1')*s2;
    a1=numerator/denominator;
    s2=switchMRP(s2);
    s1=switchMRP(s1);
    numerator = (1-(norm(s2)^2))*s1+(1-(norm(s1)^2))*s2-2*cross(s1,s2);
    denominator = 1+(norm(s1)^2)*(norm(s2)^2)-2*(s1')*s2;
    b1=numerator/denominator;
    b
    if mrpTOphi(a1)<=mrpTOphi(b1)
        amrp=a1;
    else
        amrp=b1;
    end
else
numerator = (1-(norm(s2)^2))*s1+(1-(norm(s1)^2))*s2-2*cross(s1,s2);
denominator = 1+(norm(s1)^2)*(norm(s2)^2)-2*(s1')*s2;
c1=numerator/denominator;
amrp=c1;
end
end
function phi=mrpTOphi(mrp)
q0=(1-(norm(mrp)^2))/(1+(norm(mrp)^2));
phi=2*acos(q0);
end