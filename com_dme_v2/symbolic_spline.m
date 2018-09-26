function [tp,tv,ta] = symbolic_spline(init,fin,nen)
global dt simTime
com_init_pos=init(1:3);
com_init_vel=init(4:6);
com_init_acc=init(7:9);
com_final_pos=fin(1:3);
com_final_vel=fin(4:6);
com_final_acc=fin(7:9);
%com_final_vel=zeros(3,1);%fin(4:6);
%com_final_acc=zeros(3,1);%fin(7:9);
tf=init(end); %simTime for 2 knotpoints

x0 = com_init_pos(1);
dx0 = com_init_vel(1);
ddx0 = com_init_acc(1);
x1 = com_final_pos(1);
dx1 = com_final_vel(1);
ddx1 = com_final_acc(1);

y0 = com_init_pos(2);
dy0 = com_init_vel(2);
ddy0 = com_init_acc(2);
y1 = com_final_pos(2);
dy1 = com_final_vel(2);
ddy1 = com_final_acc(2);

ax = -(12*x0 - 12*x1 + 6*dx0*tf + 6*dx1*tf + ddx0*tf^2 - ddx1*tf^2)/(2*tf^5);
bx = (30*x0 - 30*x1 + 16*dx0*tf + 14*dx1*tf + 3*ddx0*tf^2 - 2*ddx1*tf^2)/(2*tf^4);
cx = -(20*x0 - 20*x1 + 12*dx0*tf + 8*dx1*tf + 3*ddx0*tf^2 - ddx1*tf^2)/(2*tf^3);
dx = ddx0/2;
ex = dx0;
fx = x0;

ay = -(12*y0 - 12*y1 + 6*dy0*tf + 6*dy1*tf + ddy0*tf^2 - ddy1*tf^2)/(2*tf^5);
by = (30*y0 - 30*y1 + 16*dy0*tf + 14*dy1*tf + 3*ddy0*tf^2 - 2*ddy1*tf^2)/(2*tf^4);
cy = -(20*y0 - 20*y1 + 12*dy0*tf + 8*dy1*tf + 3*ddy0*tf^2 - ddy1*tf^2)/(2*tf^3);
dy = ddy0/2;
ey = dy0;
fy = y0;

if nen
  t=0:dt:tf-dt;
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