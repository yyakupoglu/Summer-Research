function plotxy(a)
global robot
for p = 1:100:length(a)
    b = centerOfMass(robot,a(:,p));
    plot(b(1),b(2),'bo')
    axis([-2 2 -2 2])
    hold on
    pause(1)
end
end