function plotxy_from_q(a)
global robot
hold on
step =1;
if size(a,2)>100
    step = 100;
end
for p = 1:step:size(a,2)
    b = centerOfMass(robot,[a(:,p)]);
    plot(b(1),b(2),'mo')
end
end