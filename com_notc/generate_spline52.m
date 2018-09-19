function [tp,tv,ta] = generate_spline52(init,fin,endknot)

com_init_pos=init(1:3);
com_init_acc=init(4:6);
com_init_vel=[0; 0; 0];
com_final_pos=fin(1:3);
com_final_acc=fin(4:6);
com_final_vel=[0; 0; 0];
tf=init(end);
dt=0.0001;
t=0:dt:tf;

% 5th order trajectory

fx = com_init_pos(1);
fy = com_init_pos(2);

ex = com_init_vel(1);
ey = com_init_vel(2);

dx = com_init_acc(1)/2;
dy = com_init_acc(2)/2;

x = [(com_final_pos(1)-ex*tf-fx)/(tf^3);
     (com_final_vel(1)-2*dx*tf-ex)/(tf^2);
     (com_final_acc(1)-2*dx)/(tf)];
 
y = [(com_final_pos(2)-ey*tf-fy)/(tf^3);
     (com_final_vel(2)-2*dy*tf-ey)/(tf^2);
     (com_final_acc(2)-2*dy)/(tf)];
A = [ tf^2 tf 1;
      5*tf^2 4*tf 3;
      20*tf^2 12*tf 6];
vx = linsolve(A,x);
vy = linsolve(A,y);

ax = vx(1);
bx = vx(2);
cx = vx(3);
ay = vy(1);
by = vy(2);
cy = vy(3);

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
