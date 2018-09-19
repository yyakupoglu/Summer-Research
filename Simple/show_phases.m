global N
for i = 1:13:length(xOpt)
    plot(xOpt(i),xOpt(i+1),'o')
    hold on
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints']);