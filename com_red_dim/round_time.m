function t = round_time(t,pos_h)
global simTime
decimal_of_dt = 3;
rounded_sum = 0;
for i=1:length(t)-2
    exact_time = t{i}(pos_h);
    rounded_time = round(exact_time,decimal_of_dt);

    rounded_sum = rounded_sum + rounded_time;
    t{i}(pos_h) = rounded_time;
end
t{length(t)-1}(pos_h) = round(simTime-rounded_sum ,decimal_of_dt);
end