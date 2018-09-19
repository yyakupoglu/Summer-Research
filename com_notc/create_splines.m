function [mainTrajectory_pos,mainTrajectory_vel,mainTrajectory_acc] = create_splines(knots)
mainTrajectory_pos=[];
mainTrajectory_vel=[];
mainTrajectory_acc=[];
spline_cond = cell(1,length(knots));
spline_cond{1} = [knots{1}(7:12); knots{1}(end)];
flag = 1;
for i = 2:length(knots)
    if i == length(knots)
        flag = 0;
    end
    spline_cond{i} = [knots{i}(7:12); knots{i}(end)];
    [splp, splv, spla] = generate_spline53(spline_cond{i-1},spline_cond{i},flag);
    mainTrajectory_pos = [mainTrajectory_pos splp];
    mainTrajectory_vel = [mainTrajectory_vel splv];
    mainTrajectory_acc = [mainTrajectory_acc spla];
    plot(splp(1,:),splp(2,:));
    axis([-2 2 -5 5]);
    view(2);
    hold on
    pause(1);
end
end