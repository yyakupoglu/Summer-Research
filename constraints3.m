function [cineq,c]=constraints3(x, robot,IC,FC,knotPoints)
cineq=[];
c=[];
m=2.5;
g=-9.81;
h0=1;
for i = 1: knotPoints
    %Decision Variables
    q = [x(25*(i-1)+1); x(25*(i-1)+2);x(25*(i-1)+3)];
    v = [x(25*(i-1)+4); x(25*(i-1)+5);x(25*(i-1)+6)]; %dq
    r = [x(25*(i-1)+7); x(25*(i-1)+8);x(25*(i-1)+9)];
    dr = [x(25*(i-1)+10); x(25*(i-1)+11);x(25*(i-1)+12)];
    ddr = [x(25*(i-1)+13); x(25*(i-1)+14);x(25*(i-1)+15)];
    lambda = [x(25*(i-1)+16); x(25*(i-1)+17);x(25*(i-1)+18)];
    k = [x(25*(i-1)+19); x(25*(i-1)+20);x(25*(i-1)+21)];
    dk = [x(25*(i-1)+22); x(25*(i-1)+23);x(25*(i-1)+24)];
    h = [x(25*(i-1)+25)];
    
    
    %CoM acceleration
    c=[c;m*ddr(1)-lambda(1);m*ddr(2)-lambda(2)-m*g;m*ddr(3)-lambda(3)];
    
    %Centroidal Anguar Momentum
    A_G=getCentroidalMomentumMatrix(robot,q);
    c=[c;k(1)-A_G(1,:)*v; k(2)-A_G(2,:)*v; k(3)-A_G(3,:)*v];
    
    %Centroidal Angular Momentum derivative
    cr=cross(r,lambda);
    c=[c; dk(1)+cr(1); dk(2)+cr(2);dk(3)+cr(3)];
    
    
    %Center of Mass Position
    p=centerOfMass(robot,q);
    c=[c; r(1)-p(1); r(2)-p(2);r(3)-p(3)];
    
end

%Time Integration Constraints for initial condition

%Joint Velocity

c=[c;x(1:3)-IC(1:3)-h0*x(4:6)];


%Angular Momentum Derivative
c=[c;x(19:21)-IC(19:21)-h0*x(22:24)];


%Center of Mass Acceleration
c=[c;x(10:12)-IC(10:12)-h0*x(13:15)];

%Mid Point CoM Velocity
c=[c;2*(x(7:9)-IC(7:9))-h0*(x(10:12)+IC(10:12))];

for i =1:knotPoints
    %Time Integration Constraints for other knots
    if i == knotPoints
        %Final Conditions
        %Joint Velocity
        c=[c;FC(1:3)-x(25*(i-1)+1:25*(i-1)+3)-x(25*(i-1)+25)*FC(4:6)];
        
        %Angular Momentum Derivative
        c=[c;FC(19:21)-x(25*(i-1)+19:25*(i-1)+21)-x(25*(i-1)+25)*FC(22:24)];
        
        %Center of Mass Acceleration
        c=[c;FC(10:12)-x(25*(i-1)+10:25*(i-1)+12)-x(25*(i-1)+25)*FC(13:15)];
        
        %Mid Point CoM Velocity
        c=[c;2*(FC(7:9)-x(25*(i-1)+7:25*(i-1)+9))-x(25*(i-1)+25)*(FC(10:12)+x(25*(i-1)+10:25*(i-1)+12))];
            
    else
        %Joint Velocity
        c=[c;x(25*i+1:25*i+3)-x(25*(i-1)+1:25*(i-1)+3)-x(25*(i-1)+25)*x(25*i+4:25*i+6)];


        %Angular Momentum Derivative
        c=[c;x(25*i+19:25*i+21)-x(25*(i-1)+19:25*(i-1)+21)-x(25*(i-1)+25)*x(25*i+22:25*i+24)];


        %Center of Mass Acceleration
        c=[c;x(25*i+10:25*i+12)-x(25*(i-1)+10:25*(i-1)+12)-x(25*(i-1)+25)*x(25*i+13:25*i+15)];

        %Mid Point CoM Velocity
        c=[c;2*(x(25*i+7:25*i+9)-x(25*(i-1)+7:25*(i-1)+9))-x(25*(i-1)+25)*(x(25*i+10:25*i+12)+x(25*(i-1)+10:25*(i-1)+12))];
    end
    
end
c=[c; x(25)+x(50)+x(75)+x(100)+x(125)-5];
cineq=[-x(25);-x(50);-x(75);-x(100);-x(125)];
end
