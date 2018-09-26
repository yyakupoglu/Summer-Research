function t = calculate_time_integration_values(t)
global m g N

s = size(t,1);
t(s+1:s+9,1) = zeros(9,1);
%%t{length(t)}(s+1:s+9) = zeros(9,1);
for i = 2:N
    ddr = [0; -m*g;0] + t(7:9,i);
    t(s+1:s+3,i) = ddr;
    t(s+4:s+6,i) = t(10,i-1)*ddr+t(s+4:s+6,i-1);
    t(s+7:s+9,i) = (t(1:3,i-1)-t(1:3,i-1))/t(10,i-1);
end