function show_robot(xOpt)
global N robot simTime
hold on
for i = 1:16:length(xOpt)
    show(robot,xOpt(i:i+2))
    hold on
    plot(xOpt(i+6),xOpt(i+7),'ro')
    pause(1)
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints Total Time=' num2str(simTime) ' s']);

end