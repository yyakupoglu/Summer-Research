function [cineq,c]=constraints2(x, robot,IC,FC,knotPoints)
cineq=[];
c=[];
m=2.5;
g=-9.81;
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
    
    if i == 1
        r = IC(7:9);
        dr =IC(10:12);
        ddr=IC(13:15);
    elseif i == knotPoints
        r = FC(7:9);
        dr =FC(10:12);
        ddr=FC(13:15);
    end
    
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
for i =1:knotPoints-1
    %Time Integration Constraints
%     if i ==1
        %Joint Velocity
        c=[c;x(25*i+1:25*i+3)-x(25*(i-1)+1:25*(i-1)+3)-x(25*(i-1)+25)*x(25*i+4:25*i+6)];
%         c(24*(i-1)+13)=q(1)-IC(1)-h*v(1);
%         c(24*(i-1)+14)=q(2)-IC(2)-h*v(2);
%         c(24*(i-1)+15)=q(3)-IC(3)-h*v(3);
        
        %Angular Momentum Derivative 
        c=[c;x(25*i+19:25*i+21)-x(25*(i-1)+19:25*(i-1)+21)-x(25*(i-1)+25)*x(25*i+22:25*i+24)];
%         c(24*(i-1)+16)=k(1)-IC(19)-h*dk(1);
%         c(24*(i-1)+17)=k(2)-IC(20)-h*dk(2);
%         c(24*(i-1)+18)=k(3)-IC(21)-h*dk(3);
        
        %Center of Mass Acceleration
        c=[c;x(25*i+10:25*i+12)-x(25*(i-1)+10:25*(i-1)+12)-x(25*(i-1)+25)*x(25*i+13:25*i+15)];
%         c(24*(i-1)+19)=dr(1)-IC(10)-h*ddr(1);
%         c(24*(i-1)+20)=dr(2)-IC(11)-h*ddr(2);
%         c(24*(i-1)+21)=dr(3)-IC(12)-h*ddr(3);
        
        %Mid Point CoM Velocity
        c=[c;2*(x(25*i+7:25*i+9)-x(25*(i-1)+7:25*(i-1)+9))-x(25*(i-1)+25)*(x(25*i+10:25*i+12)+x(25*(i-1)+10:25*(i-1)+12))];
%         c(24*(i-1)+22)=2*r(1)-2*IC(7)-h*(dr(1)+IC(10));
%         c(24*(i-1)+23)=2*r(2)-2*IC(8)-h*(dr(2)+IC(11));
%         c(24*(i-1)+24)=2*r(3)-2*IC(9)-h*(dr(2)+IC(12));
%     else
%         %Joint Angle
%         c(24*(i-1)+13)=q(1)-x(25*(i-2)+1)-h*v(1);
%         c(24*(i-1)+14)=q(2)-x(25*(i-2)+2)-h*v(2);
%         c(24*(i-1)+15)=q(3)-x(25*(i-2)+3)-h*v(3);
%         
%         %Angular Momentum
%         c(24*(i-1)+16)=k(1)-x(25*(i-2)+19)-h*dk(1);
%         c(24*(i-1)+17)=k(2)-x(25*(i-2)+20)-h*dk(2);
%         c(24*(i-1)+18)=k(3)-x(25*(i-2)+21)-h*dk(3);
%         
%         %Angular Momentum Derivative
%         c(24*(i-1)+19)=dr(1)-x(25*(i-2)+10)-h*ddr(1);
%         c(24*(i-1)+20)=dr(2)-x(25*(i-2)+11)-h*ddr(2);
%         c(24*(i-1)+21)=dr(3)-x(25*(i-2)+12)-h*ddr(3);
%         
%         %Mid Point CoM Velocity
%         c(24*(i-1)+22)=2*r(1)-2*x(25*(i-2)+7)-h*(dr(1)+x(25*(i-2)+10));
%         c(24*(i-1)+23)=2*r(2)-2*x(25*(i-2)+8)-h*(dr(2)+x(25*(i-2)+11));
%         c(24*(i-1)+24)=2*r(3)-2*x(25*(i-2)+9)-h*(dr(2)+x(25*(i-2)+12));
        
end
    c=[c; x(25)+x(50)+x(75)+x(100)+x(125)-5];
    cineq=[-x(25);-x(50);-x(75);-x(100);-x(125)];
end  
