
clear all
close all
clc
warning('off','all')
global robot m N g simTime dt 
% N stands for total knotPoints including initial and Final
N = 10;
simTime = 2;
dt = 10^(-3);
tm = 0:dt:simTime;
%% Build Robot

construct_robot();

%% Optimization

[trajectory_knots_Matrix,xOpt] = optimizecomreddim();

%% Find Time Integration variables

trajectory_knots_extended = calculate_time_integration_values(trajectory_knots_Matrix);
%% Add splines

[trajectory_pos, trajectory_vel, trajectory_acc] = create_splines_red(trajectory_knots_extended);
show_robot(xOpt,10);

%% Calculate Torque via QP nd simulate using ode45
[q_real, qdot_real, r_real,rdot_real,torques,acc_des] = solveQP(trajectory_pos, trajectory_vel, trajectory_acc);


%% Plotting
figure()
plot(tm, trajectory_pos(1,:)-r_real(1,:))
figure()
plot(r_real(1,:),r_real(2,:),'b','LineWidth',2)
hold on
plot(trajectory_knots_Matrix(4,:),trajectory_knots_Matrix(5,:),'ro','MarkerFaceColor', 'r','MarkerSize',10)
plot(trajectory_pos(1,:),trajectory_pos(2,:),'ko','MarkerFaceColor', 'k','MarkerSize',2)
plotxy_from_q([trajectory_knots_Matrix(1,:);trajectory_knots_Matrix(2,:)])
figure()
plot(tm(1:end-1),torques(1,:))
