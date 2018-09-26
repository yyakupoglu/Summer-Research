function [F,G] = conscomdmev2(x)
global robot m N g simTime dme_cost_array

 
n = 16; %decision variable in one node
gv= [0; -m*g; 0];
%[~,Jcom{1}] = centerOfMass(robot, robot.homeConfiguration);
L=10000*eye(3);
s = 3*4; % 3 dimension x 4 constraint +1 equation in the for loop
c=zeros((N-1)*s+1,1);

Cost=0;
qn = [90; 0;0]*pi/180;
%Qq= 0.001*eye(3);
Qddr =0.01*eye(3);
%rn = centerOfMass(robot, qn);
%Qr = 0.001*eye(3);
for i = 2:N
    %Decision Variables
      q = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      v = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      r = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
      dr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
      ddr = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
      
      qp = [x(n*(i-2)+1); x(n*(i-2)+2);x(n*(i-2)+3)];
      rp = [x(n*(i-2)+7); x(n*(i-2)+8);x(n*(i-2)+9)];
      drp = [x(n*(i-2)+10); x(n*(i-2)+11);x(n*(i-2)+12)];
      hp =[x(n*(i-2)+16)];
  
    %CoM position
    [p,Jcom]=centerOfMass(robot,q);
    c(s*(i-2)+1:s*(i-2)+3) = r-p;
% 
% 
    %Time Integration Constraints
    %CoM accelerarion
    c(s*(i-2)+4:s*(i-2)+6)= hp*ddr-dr+drp;
    %CoM velocity
    c(s*(i-2)+7:s*(i-2)+9)= hp*(dr+drp)-2*(r-rp);
    %Joint Velocity
    c(s*(i-2)+10:s*(i-2)+12)= hp*v-q+qp;
    % Dme Costraint
%     Jcom_dot = (dr- Jcom*q)\v;
%     iM=pinv(massMatrix(robot,q));
%     C = velocityProduct(robot,q,v);
%     rbias = -Jcom*iM*gv;%+Jcom_dot*v-Jcom*iM*C;
%     Tau=pinv((Jcom*iM*L))*(r-rbias);
%     c(s*(i-2)+13:s*(i-2)+15)= Tau;

    %Costs
    [~,Jcomp] = centerOfMass(robot,qp);
    Jcom_dot = (Jcom-Jcomp)/hp;
    iM=pinv(massMatrix(robot,q));
    C = velocityProduct(robot,q,v);
    rbias = -Jcom*iM*C-Jcom*iM*gv+Jcom_dot*v;
    Tau=pinv((Jcom*iM*L))*(r-rbias);
    dme_value = Tau'*Tau;
    %me_value = (Jcom*q)'*(Jcom*q);
    if dme_value <= 1 
        dme_cost = 0;
    else
        dme_cost = 10000000;
    end


    dme_cost_array = [dme_cost_array,dme_cost];
    Cost = Cost+hp*(ddr'*Qddr*ddr+dme_cost);
end
%Time Sum Constraint
c(end) =sum(x(n:n:end))-simTime;


 F = [ Cost;
       c];
 G= [];

end