function dydt = dynamics(t,y,T,L)
global robot m
q = [y(1); y(2); y(3)];
qd =[y(4); y(5); y(6)];
M = massMatrix(robot, q);
C=velocityProduct(robot,q,qd);
G=gravityTorque(robot,q);
[~,Jcom] = centerOfMass(robot, q);
qdd = inv(M)*(T-C-G);
dydt(1) = y(4);
dydt(2) = y(5);
dydt(3) = y(6);
dydt(4) = qdd(1);
dydt(5) = qdd(2);
dydt(6) = qdd(3);
dydt(7) = y(10);
dydt(8) = y(11);
dydt(9) = y(12);
dydt(10) = L(1)/m;
dydt(11) = L(2)/m;
dydt(12) = L(3)/m;
dydt= dydt';
end