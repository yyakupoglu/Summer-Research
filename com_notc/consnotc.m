function [F,G] = consnotc(x)
global robot m N g simTime
c=[];
 
n = 16;
gv= [0; -m*g; 0];
Jcom = cell(1,N);
[~,Jcom{1}] = centerOfMass(robot, robot.homeConfiguration);
L=1000*eye(3);
for i = 2:N
    %Decision Variables
      q = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      v = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      r = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
      ddr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
      %ddr = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
      lambda = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
      h =[x(n*(i-1)+16)];
      
      
%       qp = [x(n*(i-2)+1); x(n*(i-2)+2);x(n*(i-2)+3)];
%       rp = [x(n*(i-2)+7); x(n*(i-2)+8);x(n*(i-2)+9)];
%       drp = [x(n*(i-2)+10); x(n*(i-2)+11);x(n*(i-2)+12)];
      hp =[x(n*(i-2)+16)];
    %CoM acceleration
    c=[c;m*ddr-gv-lambda];    
    %CoM position
    [p,Jc]=centerOfMass(robot,q);
    c=[c;r-p];
    %Dynamic Manipulability Constraints
    Jcom{i}=Jc;
    Jdot= (Jcom{i}-Jcom{i-1})/hp;
    iM=pinv(massMatrix(robot,q));
    C = velocityProduct(robot,q,v);
    rbias = -Jc*iM*C-Jc*iM*gv+Jdot*v;
    TT=pinv((Jc*iM*L))*(r-rbias);
    %TT=(r-rbias)'*pinv((Jc*iM*L)*(Jc*iM*L)')*(r-rbias); %2 norm
    c=[c;TT];
    %Time Integration Constraints
%     %CoM accelerarion
%     c= [c; h*ddr-dr+drp];
%     %CoM velocity
%     c= [c; h*(dr+drp)-2*(r-rp)];
%     %Joint Velocity
%     c=[c;  h*v-q+qp];
    
end
%Time Sum Constraint
c=[c; sum(x(n:n:end))-simTime];


Qlambda =0.01*eye(3);
Cost=0;
for i = 1: N
    l_ddr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
    l_lambda=[x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
    l_h=[x(n*(i-1)+16)];
    l=l_h*(norm(l_ddr,2)^2+l_lambda'*Qlambda*l_lambda);
    Cost=Cost+l;
end

 F = [ Cost;
       c];
 G= [];
   
end