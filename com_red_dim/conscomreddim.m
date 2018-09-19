function [F,G] = conscomreddim(x)
global robot m N g simTime

 
n = 10; %decision variable
gv= [0; -m*g; 0];
[~,Jcom{1}] = centerOfMass(robot, robot.homeConfiguration);
L=1000*eye(3);
s = 3; % 3 dimension x 5 constraint equation in the for loop
c=zeros((N-1)*s+1,1);

Cost=0;
qn = [90; 0;0]*pi/180;
Qq= 0.001*eye(3);
Qlambda =0.01*eye(3);
rn = centerOfMass(robot, qn);
Qr = 0.001*eye(3);
for i = 2:N
    %Decision Variables
      q = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      r = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      lambda = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];

      hp =[x(n*(i-2)+10)];
  
    %CoM position
    [p,Jc]=centerOfMass(robot,q);
    c(s*(i-2)+1:s*(i-2)+3) = r-p;
% 
% 
%     %Time Integration Constraints
%     %CoM accelerarion
%     c(s*(i-2)+7:s*(i-2)+9)= hp*ddr-dr+drp;
%     %CoM velocity
%     c(s*(i-2)+10:s*(i-2)+12)= hp*(dr+drp)-2*(r-rp);
%     %Joint Velocity
%     c(s*(i-2)+13:s*(i-2)+15)= hp*v-q+qp;
    
    Cost = Cost+hp*(lambda'*Qlambda*lambda);
end
%Time Sum Constraint
c(end) =sum(x(n:n:end))-simTime;


 F = [ Cost;
       c];
 G= [];

end