function [xl, xu] = setStateBoundaries(n,IC,FC)
for i = 1:(n-2*length(IC))+3
    if mod(i,16) == 0
        xl(i)=1;
        xu(i)=5;
    elseif mod(i,16)==1 ||  mod(i,16)==2 || mod(i,16)==3
        xl(i)=-2*pi;
        xu(i)=2*pi;
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xl=[IC; xl';FC(4:end)];
xu=[IC; xu';FC(4:end)];
end