function [xl, xu] = setStateBoundaries(n)
for i = 1:n
    if mod(i,25) == 1 || mod(i,25) == 2 ||mod(i,25) == 3
        xl(i)=-pi;
        xu(i)=pi;
    elseif mod(i,25) == 0
        xl(i)=0;
        xu(i)=5;
    else
        xl(i)=-10000;
        xu(i)=10000;
    end
end
xl=xl';
xu=xu';
end