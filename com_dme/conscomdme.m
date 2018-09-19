function [F,G] = conscomdme(x)
tic
global robot m N g simTime

 
n = 19; %decision variable
gv= [0; -m*g; 0];
[~,Jcom{1}] = centerOfMass(robot, robot.homeConfiguration);
L=1000*eye(3);
s = 3*5; % 3 dimension x 5 constraint equation in the for loop
c=zeros((N-1)*s+1,1);

Cost=0;
qn = [90; 0;0]*pi/180;
Qq= 0.001*eye(3);
Qlambda =0.01*eye(3);

for i = 2:N
    %Decision Variables
      q = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      v = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      r = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
      dr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
      ddr = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
      lambda = [x(n*(i-1)+16); x(n*(i-1)+17);x(n*(i-1)+18)];
      %h =[x(n*(i-1)+19)];
      
      
      qp = [x(n*(i-2)+1); x(n*(i-2)+2);x(n*(i-2)+3)];
      rp = [x(n*(i-2)+7); x(n*(i-2)+8);x(n*(i-2)+9)];
      drp = [x(n*(i-2)+10); x(n*(i-2)+11);x(n*(i-2)+12)];
      hp =[x(n*(i-2)+19)];
    %CoM acceleration
    c(s*(i-2)+1:s*(i-2)+3) = m*ddr-gv-lambda;    
    %CoM position
    [p,Jc]=centerOfMass(robot,q);
    c(s*(i-2)+4:s*(i-2)+6) = r-p;
    %Dynamic Manipulability Constraints
%     Jcom{i}=Jc;
%     Jdot= (Jcom{i}-Jcom{i-1})/hp;
%     iM=pinv(massMatrix(robot,q));
%     C = velocityProduct(robot,q,v);
%     rbias = -Jc*iM*C-Jc*iM*gv+Jdot*v;
%     TT=pinv((Jc*iM*L))*(r-rbias);
%     %TT=(r-rbias)'*pinv((Jc*iM*L)*(Jc*iM*L)')*(r-rbias); %2 norm
%     c=[c;TT];
    %Time Integration Constraints
    %CoM accelerarion
    c(s*(i-2)+7:s*(i-2)+9)= hp*ddr-dr+drp;
    %CoM velocity
    c(s*(i-2)+10:s*(i-2)+12)= hp*(dr+drp)-2*(r-rp);
    %Joint Velocity
    c(s*(i-2)+13:s*(i-2)+15)= hp*v-q+qp;
    
    Cost = Cost+hp*((q-qn)'*Qq*(q-qn)+lambda'*Qlambda*lambda+ddr'*ddr+v'*v);
end
%Time Sum Constraint
c(end) =sum(x(n:n:end))-simTime;
% 
% 
% 
% 
% for i = 1: N
%     l_ddr = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
%     l_lambda=[x(n*(i-1)+16); x(n*(i-1)+17);x(n*(i-1)+18)];
%     l_h=[x(n*(i-1)+19)];
%     l=l_h*(norm(l_ddr,2)^2+l_lambda'*Qlambda*l_lambda);
%     Cost=Cost+l;
% end

 F = [ Cost;
       c];
 G= [];
 toc
end