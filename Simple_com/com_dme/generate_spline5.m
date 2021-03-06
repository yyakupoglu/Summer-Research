function [tp,tv,ta] = generate_spline5(init,fin)

com_init_pos=init(1:3);
com_init_vel=init(4:6);
com_init_acc=init(7:9);
com_final_pos=fin(1:3);
com_final_vel=fin(4:6);
com_final_acc=fin(7:9);
tf=init(end);
dt=0.001;
t=0:dt:tf;

% 5th order trajectory

fx = com_init_pos(1);
fy = com_init_pos(2);

ex = com_init_vel(1);
ey = com_init_vel(2);

dx = 0.5*com_init_acc(1);
dy = 0.5*com_init_acc(2);

cx = (20*(com_final_pos(1)-com_init_pos(1))-(8*com_final_vel(1)+12*com_init_vel(1))*tf-(3*com_final_acc(1)-com_init_acc(1))*tf^2)/(2*tf^3);
cy = (20*(com_final_pos(2)-com_init_pos(2))-(8*com_final_vel(2)+12*com_init_vel(2))*tf-(3*com_final_acc(2)-com_init_acc(2))*tf^2)/(2*tf^3);

bx = (30*(com_init_pos(1)-com_final_pos(1))+(14*com_final_vel(1)+16*com_init_vel(1))*tf+(3*com_final_acc(1)-2*com_init_acc(1))*tf^2)/(2*tf^4);
by = (30*(com_init_pos(2)-com_final_pos(2))+(14*com_final_vel(2)+16*com_init_vel(2))*tf+(3*com_final_acc(2)-2*com_init_acc(2))*tf^2)/(2*tf^4);

ax = (12*(com_final_pos(1)-com_init_pos(1))-6*(com_final_vel(1)+com_init_vel(1))*tf-(com_final_acc(1)-com_init_acc(1))*tf^2)/(2*tf^5);
ay = (12*(com_final_pos(2)-com_init_pos(2))-6*(com_final_vel(2)+com_init_vel(2))*tf-(com_final_acc(2)-com_init_acc(2))*tf^2)/(2*tf^5);


for i=1:length(t)-1
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