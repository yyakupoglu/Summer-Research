function [mainTrajectory_pos,mainTrajectory_vel,mainTrajectory_acc] = create_splines_red(knots)
notendknot =1 ;
spline_cond{1} = [knots(4:6,1);knots(14:16,1);knots(11:13,1); knots(10,1)]; %Pass r dr ddr h
spline_cond{2} = [knots(4:6,2);knots(14:16,2);knots(11:13,2); knots(10,2)];
[is_pos, is_vel, is_acc] = generate_spline5(spline_cond{1},spline_cond{2},notendknot);
mainTrajectory_pos=is_pos;%Initial Spline Positions
mainTrajectory_vel=is_vel;
mainTrajectory_acc=is_acc;
plot(is_pos(1,:),is_pos(2,:));
axis([-2 2 -2 2])
view(2);
hold on

for i = 3:size(knots,2)
    if i == size(knots,2)
        notendknot =0;
    end
    spline_cond{i} = [knots(4:6,i);knots(14:16,i);knots(11:13,i); knots(10,i)];
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
pause(1)
end