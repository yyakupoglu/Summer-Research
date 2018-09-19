function show_robot(xOpt)
global N robot simTime
hold on
for i = 1:19:length(xOpt)
    show(robot,xOpt(i:i+2))
    hold on
    plot(xOpt(i+6),xOpt(i+7),'ro')
    pause(1)
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints hT=' num2str(simTime) ]);

end