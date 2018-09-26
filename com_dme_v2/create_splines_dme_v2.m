function [mainTrajectory_pos,mainTrajectory_vel,mainTrajectory_acc] = create_splines_dme_v2(knots)
knots = round_time(knots,16); %16 = pos h
notendknot =1 ;
spline_cond{1} = knots(7:16,1); %Pass r dr ddr h
spline_cond{2} = knots(7:16,2);
[is_pos, is_vel, is_acc] = symbolic_spline(spline_cond{1},spline_cond{2},notendknot);
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
    spline_cond{i} = knots(7:16,i);
    [splp, splv, spla] = symbolic_spline([mainTrajectory_pos(:,end);mainTrajectory_vel(:,end);mainTrajectory_acc(:,end);spline_cond{i-1}(10)],spline_cond{i},notendknot);
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