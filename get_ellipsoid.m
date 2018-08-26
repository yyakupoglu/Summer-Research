function [ellip,min_man,max_man,ratio,newme] = get_ellipsoid(J_com)
J=J_com(1:2,:);
pJ=J'*inv(J*J');%pseudo J
ellip=pJ'*pJ;
%[V,D]=eig(inv(ellip));%Same Result with below
[V,D] = eig(J*J');
[U,S,VV] = svd(J);
if S(1,1)>S(2,2)
max_man=(S(1,1))*U(:,1);
min_man=(S(2,2))*U(:,2);
ratio=1/(S(1,1)/S(2,2));
else
min_man=(S(1,1))*U(:,1);
max_man=(S(2,2))*U(:,2);
ratio=1/(S(2,2)/S(1,1));
end
newme=S(1,1)*S(2,2);
end

