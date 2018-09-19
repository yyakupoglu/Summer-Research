function [tp,tv,ta] = generate_spline(init,fin)

com_init_pos=init(1:3);
com_init_vel=[0;0;0];
com_final_pos=fin(1:3);
com_final_vel=[0;0;0];

tf=init(end);
dt=0.0001;

%3rd order polynomial
dx=com_init_pos(1);
dy=com_init_pos(2);
cx=com_init_vel(1);
cy=com_init_vel(2);

% ax=(cx+2*(dx-com_final_pos(1))/tf)/(tf^2);
% ay=(cy+2*(dy-com_final_pos(2))/tf)/(tf^2);
% bx=-1.5*ax*tf-0.5*cx/tf;
% by=-1.5*ay*tf-0.5*cy/tf;
% 
bx = (3*com_final_pos(1)-2*cx*tf-3*dx)/(tf^2);
by = (3*com_final_pos(2)-2*cy*tf-3*dy)/(tf^2);
ax = (com_final_pos(1)-bx*tf^2-cx*tf-dx)/(tf^3);
ay = (com_final_pos(2)-by*tf^2-cy*tf-dy)/(tf^3);

% bx = (3*com_final_pos(1)-com_final_vel(1)*tf-2*cx*tf-3*dx)/(tf^2);
% by = (3*com_final_pos(2)-com_final_vel(2)*tf-2*cy*tf-3*dy)/(tf^2);
% ax = (com_final_pos(1)-bx*tf^2-cx*tf-dx)/(tf^3);
% ay = (com_final_pos(2)-by*tf^2-cy*tf-dy)/(tf^3);


t=0:dt:tf;

for i=1:length(t)-1
        tp(:,i)=[ax*t(i)^3+bx*t(i)^2+cx*t(i)+dx;
                 ay*t(i)^3+by*t(i)^2+cy*t(i)+dy;
                          0];
        tv(:,i)=[3*ax*t(i)^2+2*bx*t(i)+cx;
                 3*ay*t(i)^2+2*by*t(i)+cy;
                 0];
        ta(:,i)=[6*ax*t(i)+2*bx;
                 6*ay*t(i)+2*by;
                 0];     
end



end