global gridN
gridN = 20;
tic
% Minimize the simulation time
time_min = @(x) x(1);
% The initial parameter guess; 1 second, gridN positions, gridN velocities,
% gridN accelerations
x0 = [1; linspace(0,1,gridN)'; linspace(0,1,gridN)'; ones(gridN, 1) * 5];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;    ones(gridN * 2, 1) * -Inf;  ones(gridN, 1) * -10];
ub = [Inf;  ones(gridN * 2, 1) * Inf;   ones(gridN, 1) * 30];
% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');
% Solve for the best simulation time + control input
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @double_integrator_constraints, options);
% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
positions = optimal(2             : 1 + gridN);
vels      = optimal(2 + gridN     : 1 + gridN * 2);
accs      = optimal(2 + gridN * 2 : end);
% Make the plots
figure();
plot(times, accs);
title('Control Input (Acceleration) vs Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
figure();
plot(times, vels);
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
figure();
plot(times, positions);
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
disp(sprintf('Finished in %f seconds', toc));