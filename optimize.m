function [sol fval exitflag] = optimize(robot,kp)
g=9.81;
m=2.5;

IC = [robot.homeConfiguration', zeros(1,3), robot.centerOfMass',zeros(1,6),[0 m*g 0], zeros(1, 6), 1]';
FC = [zeros(1,3), zeros(1,3), [1,0,0],zeros(1,6),[0 m*g 0], zeros(1, 6),1]';
initial_guess = guess_solution(IC,FC,kp); 


opts = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');
[sol,fval,exitflag]=fmincon(@(x_l)lossfunction(x_l,kp),initial_guess,[],[],[],[],[],[],@(x)constraints3(x,robot,IC,FC,kp),opts);

end