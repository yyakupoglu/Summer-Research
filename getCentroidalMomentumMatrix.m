function A_G = getCentroidalMomentumMatrix(r,config)
I=getSystemInertia(r);
J=getSystemJacobian(r,config);
A=I*J;
X_G = getSystemCentroidal(r,config);
A_G = X_G'*A;
end