global simTime robot
y0 = [zeros(6,1); com_initial_pos; zeros(3,1)];
[t,x] = ode45(@dynamics,[0 simTime],y0)