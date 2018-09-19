function [tp,tv,ta] = generate_spline53(init,fin,endknot)

com_init_pos=init(1:3);
com_init_acc=init(4:6);
com_init_vel=[0; 0; 0];
com_final_pos=fin(1:3);
com_final_acc=fin(4:6);
com_final_vel=[0; 0; 0];
tf=init(end);
dt=0.001;


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

t=0:dt:tf;
for i=1:length(t)-endknot
        tp(:,i)=[ax*t(i)^5+bx*t(i)^4+cx*t(i)^3+dx*t(i)^2+ex*t(i)+fx;
                 ay*t(i)^5+by*t(i)^4+cy*t(i)^3+dy*t(i)^2+ey*t(i)+fy;
                          0];
        tv(:,i)=[5*ax*t(i)^4+4*bx*t(i)^3+3*cx*t(i)^2+2*dx*t(i)+ex;
                 5*ay*t(i)^4+4*by*t(i)^3+3*cy*t(i)^2+2*dy*t(i)+ey;
                 0];
        ta(:,i)=[20*ax*t(i)^3+12*bx*t(i)^2+6*cx*t(i)+2*dx;
                 20*ay*t(i)^3+12*by*t(i)^2+6*cy*t(i)+2*dy;
                 0];     
end



    end