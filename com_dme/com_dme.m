
clear all
close all
clc
warning('off','all')
global robot m N g simTime dt
% N stands for total knotPoints including initial and Final
N = 10;
simTime = 2;
dt = 0.001;
%% Build Robot

construct_robot();

%% Optimization

[trajectory_knots,xOpt] = optimizecomdme();

%% Add splines

[trajectory_pos, trajectory_vel, trajectory_acc] = create_splines(trajectory_knots);
hold on
show_robot(xOpt);

%% Calculate Torque via QP nd simulate using ode45

t = 0:dt:simTime;
q_real = robot.homeConfiguration;
qdot_real = zeros(3,1);
[com_prev,J_com_prev] =robot.centerOfMass;
r_real = com_prev;
rdot_real = zeros(3,1);
y0 = [q_real; qdot_real; r_real; rdot_real];
plot(r_real(1),r_real(2),'ro');
hold on
for i = 1:length(t)-1
    [com, J_com] = centerOfMass(robot, q_real(:,i));
    J_com_dot = (rdot_real(:,i)- J_com*q_real(:,i))\qdot_real(:,i);
    acc = get_pd_acc(trajectory_pos(:,i),r_real(:,i),trajectory_vel(:,i),rdot_real(:,i),trajectory_acc(:,i));
    [torque, lambda] = one_step_QP(q_real(:,i), qdot_real(:,i),acc, J_com, J_com_dot);
    [time,sol]=ode45(@(t,y)dynamics(t,y,torque, lambda),[0 dt],y0);
    
    q_real(:,i+1) = sol(end,1:3)';
    qdot_real(:,i+1) = sol(end,4:6)';
    r_real(:,i+1) = sol(end,7:9)';
    rdot_real(:,i+1) = sol(end,10:12)';
    
    %plot(r_real(1,i+1),r_real(2,i+1),'bo');
    fprintf('%dth iteration of %d \n',i,length(t)-1)
end

%%

options =  optimoptions('fmincon','Display','off');
com_des_pos = trajectory_pos;
com_des_vel = trajectory_vel;
com_des_acc = trajectory_acc;
Kp=5000;
Kd=500;
traj_freq=1001;
replan=0;
replancount=0;
q_real(:,1)=robot.homeConfiguration;
[com_prev,J_com_prev]=centerOfMass(robot,q_real(:,1));
com_vel_prev=[0;0;0];
qdot_real(:,1)=[0;0;0];
qddot_real(:,1)=[0;0;0];
for i=1:length(com_des_pos)-1
   
   [com_real_pos(:,i),J_com]=centerOfMass(robot,q_real(:,i)); 
   com_real_vel(:,i)=(com_real_pos(:,i)-com_prev)/dt;
   com_real_acc(:,i)=(com_real_vel(:,i)-com_vel_prev)/dt;
    [ellipsoid,min_vec(:,i),max_vec(:,i),ratio_vel(i),newme(i)]=get_ellipsoid(J_com);
    mm(i)=1/sqrt(det(ellipsoid));
    w(i)=sqrt(det(J_com(1:2,:)*J_com(1:2,:)'));
    
 
%     if (mod(i,traj_freq) == 0) || replan==1;
%           
%           [com_des_pos2,com_des_vel2,com_des_acc2]=generateTrajectory3(i,endTime,com_real_pos(:,i),com_real_vel(:,i),com_final_pos);   
%           com_des_pos=[com_des_pos(:,1:i-1) com_des_pos2];
%           com_des_vel=[com_des_vel(:,1:i-1) com_des_vel2];
%           com_des_acc=[com_des_acc(:,1:i-1) com_des_acc2];
%           replan=0;
%           plot(com_des_pos(1,:),com_des_pos(2,:))
%           pause(0.01)
%     end

   
   if i==(length(com_des_pos)-1)
        break
   end
   com_acc_pd(:,i) = com_des_acc(:,i)+Kp*(com_des_pos(:,i)-com_real_pos(:,i))+Kd*(com_des_vel(:,i)-com_real_vel(:,i)) ;
    
    
   J_com_dot  = (J_com-J_com_prev)/dt;
   J_com_prev = J_com;
   J_com23=J_com(1:2,:);
   
   J_dash=[-J_com23 eye(2)];
   %Torque limit 
   tl=1000; 
   nb=10^8;
   lb=[-nb,-nb,-nb,-nb,-nb,-tl,-tl,-tl,-nb,-nb];
   ub=[nb,nb,nb,nb,nb,tl,tl,tl,nb,nb];
   L=blkdiag(tl,tl,tl);
   
   
   
   %Dynamic Constraint Equations
   H=massMatrix(robot,q_real(:,i));
   M=blkdiag(H,eye(2)*(m));
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
   
   
   W_diag = diag([0.0 0.0 0.0 1 1 0 0 0 0.0 0.0]);
   
   
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
    plot(com_real_pos(1,:),com_real_pos(2,:));