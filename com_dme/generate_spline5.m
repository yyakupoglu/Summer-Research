function [tp,tv,ta] = generate_spline5(init,fin,nen)
global dt simTime
com_init_pos=init(1:3);
com_init_vel=init(4:6);
com_init_acc=init(7:9);
com_final_pos=fin(1:3);
com_final_vel=fin(4:6);
com_final_acc=fin(7:9);
tf=init(end); %simTime for 2 knotpoints



% 5th order trajectory

fx = com_init_pos(1);
fy = com_init_pos(2);

ex = com_init_vel(1);
ey = com_init_vel(2);

dx = com_init_acc(1)/2;
dy = com_init_acc(2)/2;

ax = (12*com_final_pos(1)-12*fx-6*com_final_vel(1)*tf-6*ex*tf+com_final_acc(1)*(tf^2)-2*dx*(tf^2))/(2*(tf^5));
ay = (12*com_final_pos(2)-12*fy-6*com_final_vel(2)*tf-6*ey*tf+com_final_acc(2)*(tf^2)-2*dy*(tf^2))/(2*(tf^5));

bx = (com_final_acc(1)*tf-2*com_final_vel(1)-10*ax*(tf^4)+2*dx*tf+ex)/(4*(tf^3));
by = (com_final_acc(2)*tf-2*com_final_vel(2)-10*ay*(tf^4)+2*dy*tf+ey)/(4*(tf^3));

cx = (com_final_acc(1)-20*ax*(tf^3)-12*bx*(tf^2)-2*dx)/(6*tf);
cy = (com_final_acc(2)-20*ay*(tf^3)-12*by*(tf^2)-2*dy)/(6*tf);


if nen
  t=0:dt:tf;
else
  t=0:dt:tf;
end
    
tp=[ax*t.^5+bx*t.^4+cx*t.^3+dx*t.^2+ex*t+fx;
    ay*t.^5+by*t.^4+cy*t.^3+dy*t.^2+ey*t+fy;
    zeros(1,length(t))];
tv=[5*ax*t.^4+4*bx*t.^3+3*cx*t.^2+2*dx*t+ex;
    5*ay*t.^4+4*by*t.^3+3*cy*t.^2+2*dy*t+ey;
    zeros(1,length(t))];
ta=[20*ax*t.^3+12*bx*t.^2+6*cx*t+2*dx;
    20*ay*t.^3+12*by*t.^2+6*cy*t+2*dy;
    zeros(1,length(t))]; 
 


    end