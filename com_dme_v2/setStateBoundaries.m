function [xL, xU] = setStateBoundaries(n,IC,FC)
global simTime N
p=n/N;
for i = 1:(n-2*length(IC))+3
    if mod(i,p) == 0
        xl(i)=simTime/(N-1)/sqrt(N);
        xu(i)=sqrt(N)*simTime/(N-1);
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xL=[IC(1:end-1);simTime/(N-1)/sqrt(N); xl';FC(4:end)];
xU=[IC(1:end-1);sqrt(N)*simTime/(N-1); xu';FC(4:end)];
end