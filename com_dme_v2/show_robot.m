function show_robot(x,d)
global N robot simTime
for i = 1:d:length(x)
    view(2);
    show(robot,x(i:i+2))
    view(2);
    hold on
    plot(x(i+3),x(i+4),'ro')
    axis([-1 3 -1 3])
    pause(1/N)
end
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints hT=' num2str(simTime) ]);

end