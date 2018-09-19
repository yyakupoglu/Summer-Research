function [F,G] = cons_com(x)
global robot m N g
c=[];
 
n = 16;
gv= [0; -m*g; 0];

for i = 2: N
    %Decision Variables
      q = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      r = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      dr = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
      ddr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
      lambda = [x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
      h =[x(n*(i-1)+16)];
      
      
      rp = [x(n*(i-2)+4); x(n*(i-2)+5);x(n*(i-2)+6)];
      drp = [x(n*(i-2)+7); x(n*(i-2)+8);x(n*(i-2)+9)];
    %CoM acceleration
    c=[c;m*ddr-gv-lambda];    
    %CoM position
    p=centerOfMass(robot,q);
    c=[c;r-p];
    %Time Integration Constraints
    %CoM accelerarion
    c= [c; h*ddr-dr+drp];
    %CoM velocity
    c= [c; h*(dr+drp)-2*(r-rp)];
    
end
%Time Sum Constraint
c=[c; sum(x(n:n:end))-10];


Qlambda =0.01*eye(3);
L=0;
for i = 1: N
    l_ddr = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
    l_lambda=[x(n*(i-1)+13); x(n*(i-1)+14);x(n*(i-1)+15)];
    l_h=[x(n*(i-1)+16)];
    l=l_h*(norm(l_ddr,2)^2+l_lambda'*Qlambda*l_lambda);
    L=L+l;
end

 F = [ L;
       c];
 G= [];
   
end