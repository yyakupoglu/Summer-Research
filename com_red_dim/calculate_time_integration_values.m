function t = calculate_time_integration_values(t)
global m g
t = round_time(t,10);
s = length(t{1});
t{1}(s+1:s+9) = zeros(9,1);
%%t{length(t)}(s+1:s+9) = zeros(9,1);
for i = 2:length(t)
    ddr = [0; -m*g;0] + t{i}(7:9);
    t{i}(s+1:s+3) = ddr;
    t{i}(s+4:s+6) = t{i-1}(10)*ddr+t{i-1}(s+4:s+6);
    t{i}(s+7:s+9) = (t{i}(1:3)-t{i-1}(1:3))/t{i-1}(10);
end