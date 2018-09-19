function [mainTrajectory_pos,mainTrajectory_vel,mainTrajectory_acc] = create_splines_red(knots)
notendknot =1 ;
spline_cond{1} = [knots{1}(4:6);knots{1}(14:16);knots{1}(11:13); knots{1}(10)]; %Pass r dr ddr h
spline_cond{2} = [knots{2}(4:6);knots{2}(14:16);knots{2}(11:13); knots{2}(10)];
[is_pos, is_vel, is_acc] = generate_spline5(spline_cond{1},spline_cond{2},notendknot);
mainTrajectory_pos=is_pos;%Initial Spline Positions
mainTrajectory_vel=is_vel;
mainTrajectory_acc=is_acc;
plot(is_pos(1,:),is_pos(2,:));
axis([-2 2 -2 2])
view(2);
hold on

for i = 3:length(knots)
    if i == length(knots)
        notendknot =0;
    end
    spline_cond{i} = [knots{i}(4:6);knots{i}(14:16);knots{i}(11:13); knots{i}(10)];
    [splp, splv, spla] = generate_spline5([mainTrajectory_pos(:,end);mainTrajectory_vel(:,end);mainTrajectory_acc(:,end);spline_cond{i-1}(10)],spline_cond{i},notendknot);
    mainTrajectory_pos = [mainTrajectory_pos splp];
    mainTrajectory_vel = [mainTrajectory_vel splv];
    mainTrajectory_acc = [mainTrajectory_acc spla];
    plot(splp(1,:),splp(2,:));
	axis([-2 2 -2 2])
    view(2);
    hold on
    pause(0.01);
end
end