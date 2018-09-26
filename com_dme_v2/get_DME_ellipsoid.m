function dme_value = get_DME_ellipsoid(J_com,L,H,C,G,k,ddrs)
B = J_com*inv(H)
xddot_vel=-B*C+k;
xddot_grav=-B*G;
xddot_bias=xddot_vel+xddot_grav;
JML = pinv(B*L);
dme_value = (ddrs-xddot_bias)'*JML'*JML*(ddrs-xddot_bias)
end