clear all
close all
clc
warning('off','all')
global robot m N g
% N stands for knotPoints in the middle
%% Parameters
L1 = 1;
L2 = 1;
L3 = 0.5;
m1=1;
m2=1;
m3=0.5;
m=m1+m2+m3;
r=0.01;
g = 9.81;
startTime=0;
endTime=1;
dt=0.001;
t=startTime:dt:endTime;
N=10;
%% Create Bodies
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',4);
body1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.HomePosition=130*pi/180;
setFixedTransform(joint1,trvec2tform([0, 0, 0]));
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
body1.Mass=m1;
body1.CenterOfMass=[L1/2 0 0];%In link coordinates
%body1.Inertia= convertInertiaMatrixtoVector(calculateLinkInertiaAtFrame(body1, L1, r));
addBody(robot, body1, 'base');

body2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([L1, 0, 0]));
joint2.JointAxis = [0 0 1];
joint2.HomePosition=0*pi/180;
body2.Joint = joint2;
body2.Mass=1;
body2.CenterOfMass=[L2/2 0 0];
%body2.Inertia= convertInertiaMatrixtoVector(calculateLinkInertiaAtFrame(body2, L2, r));
addBody(robot, body2, 'link1');

body3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3','revolute');
setFixedTransform(joint3, trvec2tform([L2, 0, 0]));
joint3.JointAxis = [0 0 1];
joint3.HomePosition=-20*pi/180;
body3.Joint = joint3;
body3.Mass=0.5;
body3.CenterOfMass=[L3/2 0 0];
%body3.Inertia= convertInertiaMatrixtoVector(calculateLinkInertiaAtFrame(body3, L3, r));
addBody(robot, body3, 'link2');

body4 = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body4.Joint = joint;
body4.Mass=0;
%body4.Inertia= [0 0 0 0 0 0];
addBody(robot, body4, 'link3');


robot.Gravity=[0  -g 0];
disp('Robot Constructed')
%% Optimization

[xOpt,F,xMul,Fmul,INFO]=optimizesimple()
save('xsimple.mat','xOpt');


