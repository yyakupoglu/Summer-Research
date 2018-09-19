function show_robot(xOpt)
global N robot
hold on
for i = 1:19:length(xOpt)
    show(robot,xOpt(i:i+2))
    hold on
    plot(xOpt(i+6),xOpt(i+7),'ro')
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints hT=10 0.1<h<5' ]);
show(robot,xOpt(i:i+2))
end