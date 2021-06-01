function [z,dr]=NDOrH(I,dt,w,z,M)

A=[-.5 0 0;0 .3 0;0 0 2];
        L=[.7 0 0; 0 .3 0;0 0 2];
        g2x=inv(I);
        C=[1 0 0;0 1 0; 0 0 1];
        fx=-inv(I)*(cross(w,I*w));
        g1x=inv(I);
        px=L*w;
        z_dot=(A-L*g2x*C)*z+A*px-L*[g2x*px+fx+g1x*M];
        z=z_dot*dt+z;
        zenta=z+px;
        dr=C*zenta;
end  