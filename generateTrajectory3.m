function [tp,tv,ta] = generateTrajectory3(timeIndex,tf, com_init_pos,com_init_vel,com_final_pos)

dt=0.001;
tf=tf-timeIndex*dt;
%3rd order polynomial
dx=com_init_pos(1);
dy=com_init_pos(2);
cx=com_init_vel(1);
cy=com_init_vel(2);
ax=(cx+2*(dx-com_final_pos(1))/tf)/(tf^2);
ay=(cy+2*(dy-com_final_pos(2))/tf)/(tf^2);
bx=-1.5*ax*tf-0.5*cx/tf;
by=-1.5*ay*tf-0.5*cy/tf;



t=0:dt:tf

%Trajectory Position
for i=1:length(t) 
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

% %Trajectory Velocity
% for i=1:length(t)-1
%         
% end
% tv(:,length(t))=[0;0;0];
% 
% %Trajectory Acceleration
% for i=1:length(t)-1
%         ta(:,i)=(tv(:,i+1)-tv(:,i))/dt;
% end
% ta(:,length(t))=[0;0;0];

% %Trajectory Position
% for i=1:length(t) 
%     if i>timeIndex
%         tp(:,i)=[ax*t(i)^3+bx*t(i)^2+cx*t(i)+dx;
%                  ay*t(i)^3+by*t(i)^2+cy*t(i)+dy;
%                           0];
%     elseif i==timeIndex
%         tp(:,i)=[com_init_pos];
%     else
%         tp(:,i)=[0;0;0];
%     end
% end
% 
% %Trajectory Velocity
% for i=1:length(t)-1
%     if i>=timeIndex
%         tv(:,i)=(tp(:,i+1)-tp(:,i))/dt;
%     else
%         tv(:,i)=[0;0;0];
%     end
% end
% tv(:,length(t))=[0;0;0];
% 
% %Trajectory Acceleration
% for i=1:length(t)-1
%     if i>=timeIndex
%         ta(:,i)=(tv(:,i+1)-tv(:,i))/dt;
%     else
%         ta(:,i)=[0;0;0];
%     end
% end;
% ta(:,length(t))=[0;0;0];

end

