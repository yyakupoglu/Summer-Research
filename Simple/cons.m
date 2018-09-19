function [F,G] = cons(x)
global robot m N g
c=[];
 
n = 13;
gv= [0; -m*g; 0];

for i = 2: N
    %Decision Variables
      r = [x(n*(i-1)+1); x(n*(i-1)+2);x(n*(i-1)+3)];
      dr = [x(n*(i-1)+4); x(n*(i-1)+5);x(n*(i-1)+6)];
      ddr = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
      lambda = [x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
      h =[x(n*(i-1)+13)];
      
      rp = [x(n*(i-2)+1); x(n*(i-2)+2);x(n*(i-2)+3)];
      drp = [x(n*(i-2)+4); x(n*(i-2)+5);x(n*(i-2)+6)];
    %CoM acceleration
    c=[c;m*ddr-gv-lambda];    
    
    %Time Integration Constraints
    %CoM accelerarion
    c= [c; h*ddr-dr+drp];
    %CoM velocity
    c= [c; h*(dr+drp)-2*(r-rp)];
    
end
%Time Sum Constraint
c=[c; sum(x(13:13:end))-5];


Qlambda =0.01*eye(3);
L=0;
for k = 1: N
    l_ddr = [x(n*(i-1)+7); x(n*(i-1)+8);x(n*(i-1)+9)];
    l_lambda=[x(n*(i-1)+10); x(n*(i-1)+11);x(n*(i-1)+12)];
    l_h=[x(n*(i-1)+12)];
    l=l_h*(norm(l_ddr,2)^2+l_lambda'*Qlambda*l_lambda);
    L=L+l;
end

 F = [ L;
       c];
 G= [];
   
end