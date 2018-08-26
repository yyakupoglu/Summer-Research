clear all
close all
clc
warning('off','all')
options =  optimoptions('fmincon','Display','off');

%% Parameters
L1 = 1;
L2 = 1;
L3 = 0.5;
m1=1;
m2=1;
m3=0.5;


startTime=0;
endTime=1;
dt=0.001;
t=startTime:dt:endTime;

%% Create Bodies
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',4);
body1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.HomePosition=130*pi/180;
setFixedTransform(joint1,trvec2tform([0, 0, 0]));
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
body1.Mass=1;
body1.CenterOfMass=[L1/2 0 0];
body1.Inertia= [1 1 (body1.Mass*(L1)^2)/3 0 0 0] ;

addBody(robot, body1, 'base');

body2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([L1, 0, 0]));
joint2.JointAxis = [0 0 1];
joint2.HomePosition=0*pi/180;
body2.Joint = joint2;
body2.Mass=1;
body2.CenterOfMass=[L2/2 0 0];
body2.Inertia= [1 1 (body2.Mass*(L2)^2)/3 0 0 0]
addBody(robot, body2, 'link1');

body3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3','revolute');
setFixedTransform(joint3, trvec2tform([L2, 0, 0]));
joint3.JointAxis = [0 0 1];
joint3.HomePosition=-20*pi/180;
body3.Joint = joint3;
body3.Mass=0.5;
body3.CenterOfMass=[L3/2 0 0];
body3.Inertia= [1 1 (body3.Mass*(L3)^2)/3 0 0 0];
addBody(robot, body3, 'link2');

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
body.Mass=0;
body.Inertia= [0 0 0 0 0 0];
addBody(robot, body, 'link3');


robot.Gravity=[0  -9.81 0];




%% Set Initials
q_real(:,1)= robot.homeConfiguration
qdot_real(:,1)=[0;0;0];
qddot_real(:,1)=[0;0;0];
[com_prev,J_com_prev]=centerOfMass(robot,q_real(:,1));

com_init_pos=[com_prev(1);com_prev(2);com_prev(3)];
com_init_vel=[0;0;0];
com_vel_prev=com_init_vel;
com_init_acc=[0;0;0];
com_final_pos=[0.5;0;0];
timeIndex=0;

%% CoM First Trajectory Calculation 
%Position, Velocity,Acceleration
[com_des_pos,com_des_vel,com_des_acc]=generateTrajectory3(timeIndex, endTime, com_init_pos,com_init_vel,com_final_pos);
 figure(1)
 plot(com_des_pos(1,:),com_des_pos(2,:))
 hold on
%% Calculate Torque via QP

Kp=200;
Kd=10;
traj_freq=1001;
replan=0;
replancount=0;
for i=1:length(t)-1
   
   [com_real_pos(:,i),J_com]=centerOfMass(robot,q_real(:,i)); 
   com_real_vel(:,i)=(com_real_pos(:,i)-com_prev)/dt;
   com_real_acc(:,i)=(com_real_vel(:,i)-com_vel_prev)/dt;
    [ellipsoid,min_vec(:,i),max_vec(:,i),ratio_vel(i),newme(i)]=get_ellipsoid(J_com);
    mm(i)=1/sqrt(det(ellipsoid));
    w(i)=sqrt(det(J_com(1:2,:)*J_com(1:2,:)'));
    %value(i)=com_real_pos(1:2,i)'*J_com(1:2,:)*J_com(1:2,:)'*com_real_pos(1:2,i);
    %not sure if makes sense
    %value2(i)=com_real_pos(1:2,i)'*inv(J_com(1:2,:)*J_com(1:2,:)')*com_real_pos(1:2,i);
 
    if (mod(i,traj_freq) == 0) || replan==1;
          
          [com_des_pos2,com_des_vel2,com_des_acc2]=generateTrajectory3(i,endTime,com_real_pos(:,i),com_real_vel(:,i),com_final_pos);   
          com_des_pos=[com_des_pos(:,1:i-1) com_des_pos2];
          com_des_vel=[com_des_vel(:,1:i-1) com_des_vel2];
          com_des_acc=[com_des_acc(:,1:i-1) com_des_acc2];
          replan=0;
          plot(com_des_pos(1,:),com_des_pos(2,:))
          pause(0.01)
    end

   
   if i==(length(t)-1)
        break
   end
   com_acc_pd(:,i) = com_des_acc(:,i)+Kp*(com_des_pos(:,i)-com_real_pos(:,i))+Kd*(com_des_vel(:,i)-com_real_vel(:,i)) ;
    
    
   J_com_dot  = (J_com-J_com_prev)/dt;
   J_com_prev = J_com;
   J_com23=J_com(1:2,:);
   
   J_dash=[-J_com23 eye(2)];
   %Torque limit 
   tl=50; 
   nb=10^8;
   lb=[-nb,-nb,-nb,-nb,-nb,-tl,-tl,-tl,-nb,-nb];
   ub=[nb,nb,nb,nb,nb,tl,tl,tl,nb,nb];
   L=blkdiag(tl,tl,tl);
   
   
   
   %Dynamic Constraint Equations
   H=massMatrix(robot,q_real(:,i));
   M=blkdiag(H,eye(2)*(m1+m2+m3));
   C=velocityProduct(robot,q_real(:,i),qdot_real(:,i));
   G=gravityTorque(robot,q_real(:,i));
   hh=C+G;
   h = [hh;
         0;
         0];
     
S =[1 0 0 0 0;
       0 1 0 0 0
       0 0 1 0 0];
   
   Jc=[0 0 0 1 0;
       0 0 0 0 1];
   
   A=[M -S' -Jc';
      J_dash zeros([2 5])];

   
   k=J_com_dot(1:2,:)*qdot_real(:,i);
   
   B=[-h;k];
   
   %Bias vector
   xddot_vel=-J_com23*inv(H)*C+k;
   xddot_grav=-J_com23*inv(H)*G;
   xddot_bias(:,i)=xddot_vel+xddot_grav;
   JML=pinv(J_com23*inv(H)*L);
   [dmellipsoid,min_vec_dme(:,i),max_vec_dme(:,i)]=get_dmellipsoid(J_com23,L,H,C,G,k,com_acc_pd(1:2,i));
   ellipse_overshoot(i)=(com_acc_pd(1:2,i)-xddot_bias(:,i))'*JML'*JML*(com_acc_pd(1:2,i)-xddot_bias(:,i));
   ellipse_overshoot_bias(i)=(-xddot_bias(:,i))'*JML'*JML*(-xddot_bias(:,i));
   overshoot=1.1;
   com_reduced_acc(:,i)=com_acc_pd(1:2,i);
%    if ellipse_overshoot(i)>1;
%        if ellipse_overshoot_bias(i)<1;% Means there are accelerations of which if we reduce enough we won't be saturating.
%            while overshoot>1
%                com_reduced_acc(:,i)=0.99*com_reduced_acc(:,i);
%                overshoot=(com_reduced_acc(:,i)-xddot_bias(:,i))'*JML'*JML*(com_reduced_acc(:,i)-xddot_bias(:,i));
%            end
%        else
%            com_reduced_acc(:,i)=[0;0];
% %        [tmax,tmin,com_acc_pd_new_norm]=get_directional_acc(com_acc_pd(1:2,i),min_vec_dme(:,i),max_vec_dme(:,i))
% %        com_acc_pd(1:2,i)=com_acc_pd(1:2,i)/com_acc_pd_new_norm;
%        end
%    end
   
   f=-[0;0;0;com_reduced_acc(:,i);0;0;0;0;0];
   
   
   W_diag = diag([0.0 0.0 0.0 1 1 0.0 0.0 0.0 0.0 0.0]);
   
   
   [x,fval,exitflag,output,lambda]=quadprog(W_diag,f,[],[],A,B,lb,ub,[],options);
   
%    Q=H*inv(L*L)*H;
%    Jq=inv(Q)*J_com23'*inv(J_com23*inv(Q)*J_com23'); %Jq=Jqpinv in 
%    N=Jq'*Q*Jq;
%    JML_a=J_com23*inv(H)*L;
%    w_dme(i)=sqrt(det(JML_a*JML_a'));
  
   jointTorques(:,i)=[x(6);x(7);x(8)]; 
   if int8(abs(x(6)))>=tl || int8(abs(x(7)))>=tl || int8(abs(x(8)))>=tl
       saturated(i)=1;
%         newvec=get_directional_acc(com_acc_pd(:,i),min_vec_dme(:,i),max_vec_dme(:,i));
%         f2=-[0;0;0;newvec;0;0;0;0;0]; 
%         [x,fval,exitflag,output,lambda]=quadprog(W_diag,f2,[],[],A,B,lb,ub,[],options);
%        replan=1;
%        replancount=replancount+1;
   else
       saturated(i)=0;
   end

   
   
   if ellipse_overshoot(i)>1
       saturationguess(i)=1;
   end

   qddot_real(:,i+1)= [x(1);x(2);x(3)];
   qdot_real (:,i+1)= qddot_real(:,i)*dt+qdot_real(:,i);
   q_real    (:,i+1)= qdot_real(:,i)*dt+q_real(:,i);
   
   
   
   
   
   v_com_dash1(:,i)=pinv(J_com(1:2,:))*(com_real_acc(1:2,i)-k);
   value2(i)=(qddot_real(:,i)')*(H')*H*qddot_real(:,i);
   value3(i)=(H*(pinv(J_com(1:2,:))*com_des_acc(1:2,i)))'*H*(pinv(J_com(1:2,:)))*com_des_acc(1:2,i);
   value4(i)=com_des_acc(1:2,i)'*pinv(J_com(1:2,:)*inv(H'*H)*J_com(1:2,:)')*com_des_acc(1:2,i);
   karr(:,i)=norm(k);
   
   
   
   
   
   for j=1:length(x)
      if x(j) == lb(j) || x(j) == ub(j)
           fprintf('Saturation x(%d) ',j);
           fprintf('\n');
      end
   end

   
   
   com_prev=com_real_pos(:,i);
   com_vel_prev=com_real_vel(:,i);
end
    disp('QP finished')
%% Draw Robot
figure(2)
for k=1:length(com_real_pos)-1
   error_each(k)=norm(com_real_pos(1:2,k)-com_des_pos(1:2,k));
   minnorm(k)=norm([min_vec(1,k) min_vec(2,k)]) ;
   maxnorm(k)=norm([max_vec(1,k) max_vec(2,k)]);
   mindot(k)=dot(min_vec(:,k)/minnorm(k),com_real_vel(1:2,k)/norm(com_real_vel(1:2,k)));
   maxdot(k)=dot(max_vec(:,k)/maxnorm(k),com_real_vel(1:2,k)/norm(com_real_vel(1:2,k)));
   minnormdme(k)=norm([min_vec_dme(1,k) min_vec_dme(2,k)]) ;
   maxnormdme(k)=norm([max_vec_dme(1,k) max_vec_dme(2,k)]);
   mindotdme(k)=dot(min_vec_dme(:,k)/minnormdme(k),com_des_acc(1:2,k)/norm(com_des_acc(1:2,k)));
   maxdotdme(k)=dot(max_vec_dme(:,k)/maxnormdme(k),com_des_acc(1:2,k)/norm(com_des_acc(1:2,k)));
   mindotdmenormpd(k)=dot(min_vec_dme(:,k)/minnormdme(k),com_acc_pd(1:2,k)/norm(com_acc_pd(1:2,k)));
   mindotdmenormpd2(k)=dot(min_vec_dme(:,k),com_acc_pd(1:2,k)/norm(com_acc_pd(1:2,k)));
   mindotdmenormpd3(k)=dot(min_vec_dme(:,k)/minnormdme(k),com_acc_pd(1:2,k));
   maxdotdmenormpd(k)=dot(max_vec_dme(:,k)/maxnormdme(k),com_acc_pd(1:2,k)/norm(com_acc_pd(1:2,k)));
   maxdotdmenormpd2(k)=dot(max_vec_dme(:,k),com_acc_pd(1:2,k)/norm(com_acc_pd(1:2,k)));
   maxdotdmenormpd3(k)=dot(max_vec_dme(:,k)/maxnormdme(k),com_acc_pd(1:2,k));
   mindotdmepd(k)=dot(min_vec_dme(:,k),com_acc_pd(1:2,k));
   maxdotdmepd(k)=dot(max_vec_dme(:,k),com_acc_pd(1:2,k));
end
for k=1:length(com_real_pos)-2
   minnormdmegrad(k)=minnormdme(k+1)-minnormdme(k);
   maxnormdmegrad(k)=maxnormdme(k+1)-maxnormdme(k);
end
total_Error=sum(error_each)
error=norm(com_real_pos-com_des_pos(:,1:length(com_real_pos)));
plot(error_each)
xlabel('Time Step')
ylabel('Magnitude')
hold on
plot(mm)
plot(ratio_vel)
figure(3)
plot(com_real_pos(1,:),com_real_pos(2,:),'ro')
hold on
plot(com_des_pos(1,:),com_des_pos(2,:),'go')
xlabel('X Position','FontSize',18)
ylabel('Y Position','FontSize',18)
title('CoM Without Returning Information to Planner','FontSize',18)
legend({'Red-Real','Green-Desired'},'FontSize',18)
figure(5)
 view(2)
 ax = gca;
 ax.Projection = 'orthographic';
% % hold on
% axis([-10 10 -10 10])
% framesPerSecond = 15;
% r = robotics.Rate(framesPerSecond);
% save('updated_traj.mat','com_real_pos')
% save('updated_manipulability.mat','mm')
%%
for i = 1:10:length(t)-1
    
    show(robot,q_real(:,i),'PreservePlot',false);
    axis([-2 2 -2 2])
    ax = gca;
    ax.Projection = 'orthographic';
    hold on
    plot(com_des_pos(1,i),com_des_pos(2,i),'go')
    plot(com_real_pos(1,i),com_real_pos(2,i),'ro')
    hold on
    
%     plot([com_real_pos(1,i)-min_vec(1,i) com_real_pos(1,i)+min_vec(1,i)],[com_real_pos(2,i)-min_vec(2,i) com_real_pos(2,i)+min_vec(2,i)],'k')
%     plot([com_real_pos(1,i)-max_vec(1,i) com_real_pos(1,i)+max_vec(1,i)],[com_real_pos(2,i)-max_vec(2,i) com_real_pos(2,i)+max_vec(2,i)],'k')
%     plot([com_real_pos(1,i)-min_vec_dme(1,i)/100 com_real_pos(1,i)+min_vec_dme(1,i)/100],[com_real_pos(2,i)-min_vec_dme(2,i)/100 com_real_pos(2,i)+min_vec_dme(2,i)/100],'r')
%     plot([com_real_pos(1,i)-max_vec_dme(1,i)/100 com_real_pos(1,i)+max_vec_dme(1,i)/100],[com_real_pos(2,i)-max_vec_dme(2,i)/100 com_real_pos(2,i)+max_vec_dme(2,i)/100],'r')
%     plot([com_real_pos(1,i) com_real_pos(1,i)+com_acc_pd(1,i)/10],[com_real_pos(2,i) com_real_pos(2,i)+com_acc_pd(2,i)/10],'b')
    
%     plot_ellipse(com_real_pos(:,i),min_vec(:,i),max_vec(:,i),'k')
%     plot_dmellipse(com_real_pos(:,i),min_vec_dme(:,i),max_vec_dme(:,i),[0;0],'r')
    hold off
    view(2)
    ax = gca;
    ax.Projection = 'orthographic';
    drawnow

end
%%

for i = 1:3:length(t)-1
    x1=cos(q_real(1,i));
    y1=sin(q_real(1,i));
    x2=x1+cos(q_real(1,i)+q_real(2,i));
    y2=y1+sin(q_real(1,i)+q_real(2,i));
    x3=x2+0.5*cos(q_real(1,i)+q_real(2,i)+q_real(3,i));
    y3=y2+0.5*sin(q_real(1,i)+q_real(2,i)+q_real(3,i));
    
%     show(robot,q_real(:,i),'PreservePlot',false);
     
%     ax = gca;
%     ax.Projection = 'orthographic';
   
    plot(com_des_pos(1,i),com_des_pos(2,i),'go','LineWidth',3,'MarkerSize',12,'MarkerFaceColor','g')
    hold on
    grid on
    plot(com_real_pos(1,i),com_real_pos(2,i),'ro','LineWidth',3,'MarkerSize',12,'MarkerFaceColor','r')
    axis([-2 2 -1 2]);
    plot([0 x1 x2 x3],[0 y1 y2 y3],'LineWidth',5);
    plot(x1,y1,'ko','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','k');
    plot(0,0,'ko','LineWidth',3,'MarkerSize',10,'MarkerFaceColor','k');
    plot(x2,y2,'ko','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','k');
    xlabel('X Position[m]','FontSize',18)
    ylabel('Y Position[m]','FontSize',18)
%     plot([com_real_pos(1,i)-min_vec(1,i) com_real_pos(1,i)+min_vec(1,i)],[com_real_pos(2,i)-min_vec(2,i) com_real_pos(2,i)+min_vec(2,i)],'k')
%     plot([com_real_pos(1,i)-max_vec(1,i) com_real_pos(1,i)+max_vec(1,i)],[com_real_pos(2,i)-max_vec(2,i) com_real_pos(2,i)+max_vec(2,i)],'k')
%     plot([com_real_pos(1,i)-min_vec_dme(1,i)/100 com_real_pos(1,i)+min_vec_dme(1,i)/100],[com_real_pos(2,i)-min_vec_dme(2,i)/100 com_real_pos(2,i)+min_vec_dme(2,i)/100],'r')
%     plot([com_real_pos(1,i)-max_vec_dme(1,i)/100 com_real_pos(1,i)+max_vec_dme(1,i)/100],[com_real_pos(2,i)-max_vec_dme(2,i)/100 com_real_pos(2,i)+max_vec_dme(2,i)/100],'r')
%     plot([com_real_pos(1,i) com_real_pos(1,i)+com_acc_pd(1,i)/10],[com_real_pos(2,i) com_real_pos(2,i)+com_acc_pd(2,i)/10],'b')
    
%     plot_ellipse(com_real_pos(:,i),min_vec(:,i),max_vec(:,i),'k')
%     plot_dmellipse(com_real_pos(:,i),min_vec_dme(:,i),max_vec_dme(:,i),[0;0],'r')
    hold off
%     ax = gca;
%     ax.Projection = 'orthographic';
pause(0.0005)
    

end