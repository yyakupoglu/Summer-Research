function t = round_time(t,pos_h)
global simTime
decimal_of_dt = 3;
rounded_sum = 0;
for i=1:size(t,2)-2
    exact_time = t(pos_h,i);
    rounded_time = round(exact_time,decimal_of_dt);

    rounded_sum = rounded_sum + rounded_time;
    t(pos_h,i) = rounded_time;
end
t(pos_h,size(t,2)-1) = round(simTime-rounded_sum ,decimal_of_dt);
end