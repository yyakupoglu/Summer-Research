function [xl, xu] = setStateBoundaries(n,IC,FC)
global simTime N
M = length(IC);
for i = 1:(n-2*length(IC))+3
    if mod(i,16) == 0
        xl(i)=simTime/(N-1)/10;
        xu(i)=2*simTime/(N-1);
    elseif mod(i,M)==1 ||  mod(i,M)==2 || mod(i,M)==3
        xl(i)=-2*pi;
        xu(i)=2*pi;
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xl=[IC(1:end-1);simTime/(N-1)/10; xl';FC(4:end)];
xu=[IC(1:end-1);2*simTime/(N-1);   xu';FC(4:end)];
end