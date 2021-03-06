function [q_real, qdot_real, r_real,rdot_real,torques,accdes] = solveQP(trajectory_pos, trajectory_vel, trajectory_acc)
global dt simTime robot
t = 0:dt:simTime;
q_real = robot.homeConfiguration;
qdot_real = zeros(3,1);
[com_prev,J_com_prev] =robot.centerOfMass;
r_real = com_prev;
rdot_real = zeros(3,1);
y0 = [q_real; qdot_real; r_real; rdot_real];
figure()
plot(r_real(1,1),r_real(2,1),'bo');
hold on
for i = 1:length(t)-1
    [com, J_com] = centerOfMass(robot, q_real(:,i));
    J_com_dot = (rdot_real(:,i)- J_com*q_real(:,i))\qdot_real(:,i);
    acc = get_pd_acc(trajectory_pos(:,i),r_real(:,i),trajectory_vel(:,i),rdot_real(:,i),trajectory_acc(:,i));
    [torque, lambda] = one_step_QP(q_real(:,i), qdot_real(:,i),acc, J_com, J_com_dot);
    [time,sol]=ode45(@(t,y)dynamics(t,y,torque, lambda),[0 dt],[q_real(:,i); qdot_real(:,i); r_real(:,i); rdot_real(:,i)]);
    accdes(:,i) = acc;
    torques(:,i) = torque;
    q_real(:,i+1) = sol(end,1:3)';
    qdot_real(:,i+1) = sol(end,4:6)';
    r_real(:,i+1) = sol(end,7:9)';
    rdot_real(:,i+1) = sol(end,10:12)';
    
    %     plot(r_real(1,i+1),r_real(2,i+1),'bo');
    % 
    %     pause(1);
    fprintf('%dth iteration of %d \n',i,length(t)-1)
end


end