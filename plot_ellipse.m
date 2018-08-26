function plot_ellipse(com,min,max,color)
com=com(1:2);
t=linspace(0,2*pi,40);
th=atan2(min(2),min(1));
R=[cos(th) -sin(th);
   sin(th)  cos(th)]; 
nmin=norm(min);
nmax=norm(max);
for i=1:length(t)
pos(:,i)=R*[nmin*cos(t(i)); nmax*sin(t(i))];
end
for i=1:length(t)-1
plot([com(1)+pos(1,i) com(1)+pos(1,i+1)],[com(2)+pos(2,i) com(2)+pos(2,i+1)],color);
end

