function [mainTrajectory_pos,mainTrajectory_vel,mainTrajectory_acc] = create_splines(knots)
mainTrajectory_pos=[];
mainTrajectory_vel=[];
mainTrajectory_acc=[];
spline_cond{1} = [knots{1}(7:15); knots{1}(end)];
notendknot =1 ;
for i = 2:length(knots)
    if i == length(knots)
        notendknot =0;
    end
    spline_cond{i} = [knots{i}(7:15); knots{i}(end)];
    [splp, splv, spla] = generate_spline5(spline_cond{i-1},spline_cond{i},notendknot);
    mainTrajectory_pos = [mainTrajectory_pos splp];
    mainTrajectory_vel = [mainTrajectory_vel splv];
    mainTrajectory_acc = [mainTrajectory_acc spla];
    plot(splp(1,:),splp(2,:));
    axis([-2 2 -2 2])
    view(2);
    hold on
    pause(0.5);
end
end