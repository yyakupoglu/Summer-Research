function [xl, xu] = setStateBoundaries(n,IC,FC)
global simTime N
for i = 1:(n-2*length(IC))+3
    if mod(i,19) == 0
        xl(i)=simTime/(N-1)/10;
        xu(i)=10*simTime/(N-1);
    elseif mod(i,19)==1 ||  mod(i,19)==2 || mod(i,19)==3
        xl(i)=-2*pi;
        xu(i)=2*pi;
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xl=[IC(1:end-1);simTime/100; xl';FC(4:end)];
xu=[IC(1:end-1);simTime/2;   xu';FC(4:end)];
end