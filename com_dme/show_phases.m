global N
for i = 1:16:length(xOpt)
    plot(xOpt(i+3),xOpt(i+4),'o')
    hold on
end
view(2);
xlabel('X');
ylabel('Y');
title([num2str(N) ' KnotPoints']);
