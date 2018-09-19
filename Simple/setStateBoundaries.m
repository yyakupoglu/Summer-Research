function [xl, xu] = setStateBoundaries(n,IC,FC)
for i = 1:(n-2*length(IC))
    if mod(i,13) == 0
        xl(i)=0.05;
        xu(i)=5;     
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xl=[IC; xl';FC];
xu=[IC; xu';FC];
end