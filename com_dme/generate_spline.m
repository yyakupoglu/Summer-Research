function [tp,tv,ta] = generate_spline(init,fin,nen)
global dt
com_init_pos=init(1:3);
com_init_vel=init(4:6);
com_final_pos=fin(1:3);
com_final_vel=fin(4:6);

tf=init(end);

%3rd order polynomial
dx=com_init_pos(1);
dy=com_init_pos(2);
cx=com_init_vel(1);
cy=com_init_vel(2);

ax = (com_final_vel(1)*tf+2*dx-2*com_final_pos(1)+cx*tf)/(tf^3);
ay = (com_final_vel(2)*tf+2*dy-2*com_final_pos(2)+cy*tf)/(tf^3);

bx = (com_final_vel(1)-cx-3*ax*tf^2)/(2*tf);
by = (com_final_vel(2)-cy-3*ay*tf^2)/(2*tf);


if nen %not endknot
  t=0:dt:tf-dt;
else
  t=0:dt:tf;
end
    
tp=[ax*t.^3+bx*t.^2+cx*t+dx;
    ay*t.^3+by*t.^2+cy*t+dy;
    zeros(1,length(t))];
tv=[3*ax*t.^2+2*bx*t+cx;
    3*ay*t.^2+2*by*t+cy;
    zeros(1,length(t))];
ta=[6*ax*t+2*bx;
    6*ay*t+2*by;
     zeros(1,length(t))]; 
             
end