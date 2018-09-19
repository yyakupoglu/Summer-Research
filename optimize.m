function [sol fval exitflag lambda states] = optimize()


IC = [robot.homeConfiguration', zeros(1,3), robot.centerOfMass',zeros(1,6),[0 m*g 0], zeros(1, 6), 1]';
FC = [zeros(1,3), zeros(1,3), [1,0,0],zeros(1,6),[0 m*g 0], zeros(1, 6),1]';
initial_guess = guess_solution(IC,FC,kp); 


Flow = [ 0;            %lowB bound of the cost (final time)
         zeros(length(,1);  %lowB of the dynamics of x y and v (each size N)
         x0;            %equality constraint for initial x position of particle
         y0;            %equality constraint for initial y position of particle
         v0;            %equality constraint for initial v of particle 
         xf;            %equality constraint for final x position of particle
         yf ];          %equality constraint for final y position of particle

Fupp = [ L;
         zeros(3*N,1);
         x0;
         y0;
         v0;
         xf;
         yf ];
     
     
ufun=@(x_l)lossfunction(x_l,kp);
nlcon=@(x_c)constraints3(x_c,robot,IC,FC,kp);
% opts = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
%                        'MaxFunEvals', 100000, 'Display', 'iter', ...
%                        'DiffMinChange', 0.001, 'Algorithm', 'sqp');
% [sol,fval,exitflag]=fmincon(@(x_l)lossfunction(x_l,kp),initial_guess,[],[],[],[],[],[],@(x)constraints3(x,robot,IC,FC,kp));
[sol,fval,exitflag,lambda,states]=snsolve2(ufun,initial_guess,[],[],[],[],[],[],nlcon)
end