function [ddqs,ddrs,torques, lambdas,DME_value] = one_step_QP(q, dq, new_acc, J_com, Jcomdot)
global robot m g
options =  optimoptions('fmincon','Display','off');
J_dash=[-J_com eye(3)];
   
%Torque limit 
   tl=10000; 
   nb=10^8;
   lb=[-nb,-nb,-nb,-nb,-nb,-nb,-tl,-tl,-tl,-nb,-nb,-nb];
   ub=[nb,nb,nb,nb,nb,nb,tl,tl,tl,nb,nb,nb];
   L=blkdiag(tl,tl,tl);
   
   
   
%Dynamic Constraint Equations
   H=massMatrix(robot,q);
   M=blkdiag(H,eye(3)*(m));
   C=velocityProduct(robot,q,dq);
   G=gravityTorque(robot,q);
   hh=C+G;
   h = [hh;
        [0;-m*g;0]];
     
   S =[1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 1 0 0 0];
   
   Jc=[0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
   
   A=[M -S' -Jc';
      J_dash zeros([3 6])];

   
   k=Jcomdot*dq;
   
   B=[-h;k];
   
   f=-[0;0;0;new_acc;0;0;0;0;0;0];
   
   
   W_diag = diag([0.0 0.0 0.0 1 1 1 0.0 0.0 0.0 0.0 0.0 0.0]);
   
   
   [x,fval,exitflag,output,lambda]=quadprog(W_diag,f,[],[],A,B,lb,ub,[],options);
   
   ddqs = [x(1);x(2);x(3)];
   ddrs = [x(4);x(5);x(6)];
   torques = [x(7); x(8); x(9)];
   lambdas = [x(10); x(11); x(12)];
   DME_value = get_DME_ellipsoid(J_com,L,H,C,G,k,ddrs)
end