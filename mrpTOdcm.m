function dcm=mrpTOdcm(mrp)
dcm=eye(3)+((8*skew(mrp)^2-4*(1-norm(mrp)^2)*skew(mrp))/((1+norm(mrp)^2)^2));
end

function f=skew(x)

f=[0,-x(3),x(2);

x(3),0,-x(1);

-x(2),x(1),0;];
end
function sd=sdot(x)
sd=(1/4)*((1-x'*x)*eye(3)+2*skew(x)+2*(x)*x');

end