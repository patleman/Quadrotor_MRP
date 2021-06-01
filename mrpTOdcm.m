function dcm=mrpTOdcm(mrp)
dcm=eye(3)+((8*skew(mrp)^2-4*(1-norm(mrp)^2)*skew(mrp))/((1+norm(mrp)^2)^2));
end