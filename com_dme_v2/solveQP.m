function [q_real, qdot_real, r_real,rdot_real,qddot_real,rddot_real,torques,accdes,dme_values] = solveQP(trajectory_pos, trajectory_vel, trajectory_acc)
global dt simTime robot
t = 0:dt:simTime;
q_real = robot.homeConfiguration;
qdot_real = zeros(3,1);
[com_prev,J_com_prev] =robot.centerOfMass;
r_real = com_prev;
rdot_real = zeros(3,1);


for i = 1:length(t)-1
    [com, J_com] = centerOfMass(robot, q_real(:,i));
    J_com_dot = (J_com-J_com_prev)/dt;
    acc = get_pd_acc(trajectory_pos(:,i),r_real(:,i),trajectory_vel(:,i),rdot_real(:,i),trajectory_acc(:,i));
    [ddq,ddr,torque, lambda,dme] = one_step_QP(q_real(:,i), qdot_real(:,i),acc, J_com, J_com_dot);
    [time,sol]=ode45(@(t,y)dynamics(t,y,torque, lambda),[0 dt],[q_real(:,i); qdot_real(:,i); r_real(:,i); rdot_real(:,i)]);
    qddot_real(:,i) = ddq;
    rddot_real(:,i) = ddr;
    accdes(:,i) = acc;
    torques(:,i) = torque;
    dme_values(i) = dme;
    q_real(:,i+1) = sol(end,1:3)';
    qdot_real(:,i+1) = sol(end,4:6)';
    r_real(:,i+1) = sol(end,7:9)';
    rdot_real(:,i+1) = sol(end,10:12)';
    
    J_com_prev = J_com
    %     plot(r_real(1,i+1),r_real(2,i+1),'bo');
    % 
    %     pause(1);
    fprintf('%dth iteration of %d \n',i,length(t)-1)
end


end