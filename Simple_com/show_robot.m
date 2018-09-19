global N
for i = 1:16:length(xOpt)
    show(robot,xOpt(i:i+2))
    hold on
    plot(xOpt(i+3),xOpt(i+4),'ro')
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints']);
show(robot,xOpt(i:i+2))