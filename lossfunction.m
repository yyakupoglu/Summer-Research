function L=lossfunction(x_l,knotPoints)

Qv=0.1*eye(3);
Qlambda =0.01*eye(3);
L=0;
for k = 1: knotPoints
    vk = [x_l(25*(k-1)+4); x_l(25*(k-1)+5);x_l(25*(k-1)+6)];
    ddr = [x_l(25*(k-1)+13); x_l(25*(k-1)+14);x_l(25*(k-1)+15)];
    lambda=[x_l(25*(k-1)+16); x_l(25*(k-1)+17);x_l(25*(k-1)+18)];
    h=x_l(25*(k-1)+25);
    l=h*(vk'*Qv*vk+norm(ddr,2)^2+lambda'*Qlambda*lambda);
    L=L+l;
end
end