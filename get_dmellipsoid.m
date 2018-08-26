function [N,min_man,max_man,ratio,mult] = get_dmellipsoid(J_com23,L,H,C,G,k,com_acc)
xddot_vel=-J_com23*inv(H)*C+k;
xddot_grav=-J_com23*inv(H)*G;
xddot_bias=xddot_vel+xddot_grav;
Q=H*inv(L*L)*H;
Jq=inv(Q)*J_com23'*inv(J_com23*inv(Q)*J_com23'); %Jq=Jqpinv in 
N=Jq'*Q*Jq;
value5=(com_acc-xddot_bias)'*N*(com_acc-xddot_bias);
[V,D] = eig(N);

if sqrt(D(2,2))>sqrt(D(1,1))
max_man=(1/sqrt(D(1,1)))*V(:,1);
min_man=(1/sqrt(D(2,2)))*V(:,2);
ratio=sqrt(D(1,1))/sqrt(D(2,2));
else
max_man=(1/sqrt(D(2,2)))*V(:,2);
min_man=(1/sqrt(D(1,1)))*V(:,1);
ratio=sqrt(D(2,2))/sqrt(D(1,1));
end
mult=1/sqrt(D(1,1)*D(2,2));
end

