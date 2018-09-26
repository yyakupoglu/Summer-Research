
clear all
close all
clc
warning('off','all')
global robot m N g simTime dt dme_cost_array
% N stands for total knotPoints including initial and Final
N = 10;
simTime = 2;
dt = 10^(-3);
tm = 0:dt:simTime;
dme_cost_array = [];
%% Build Robot

construct_robot();

%% Optimization

[trajectory_knots_Matrix,xOpt,F] = optimizecomdmev2();
save('dme_cost_ws.mat')
%% Add splines

[trajectory_pos, trajectory_vel, trajectory_acc] = create_splines_dme_v2(trajectory_knots_Matrix);
show_robot(xOpt,16);

%% Calculate Torque via QP nd simulate using ode45
[q_real, qdot_real, r_real,rdot_real,qdd_real,rdd_real,torques,acc_des,dme_value] = solveQP(trajectory_pos, trajectory_vel, trajectory_acc);
save('dme_cost_ws.mat')

%% Plotting
figure()
plot(tm, trajectory_pos(1,:)-r_real(1,:))
figure()
plot(r_real(1,:),r_real(2,:),'b','LineWidth',2)
hold on
plot(trajectory_knots_Matrix(7,:),trajectory_knots_Matrix(8,:),'ro','MarkerFaceColor', 'r','MarkerSize',10)
plot(trajectory_pos(1,:),trajectory_pos(2,:),'ko','MarkerFaceColor', 'k','MarkerSize',2)
plotxy_from_q([trajectory_knots_Matrix(1,:);trajectory_knots_Matrix(2,:);trajectory_knots_Matrix(3,:)])
figure()
plot(tm(1:end-1),torques(1,:))
