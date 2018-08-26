clear all
close all

L=1; 
k=1000;
g=-9.8;
b=1;
m=2;
dt=0.01;
t=0:dt:100;
fi(1)=0*pi/180;
l(1)=0.8;
dfi(1)=0;
dl(1)=0;

for i=1:length(t)-1
    ddfi(i) = g*sin(fi(i))/l(i)+2*dl(i)*dfi(i)/l(i);
    ddl(i)  = g*cos(fi(i))-k*(l(i)-L)/m+(l(i)*dfi(i)^2)/m+b*dl(i)/m;
    
    dl(i+1)=dl(i)+ddl(i)*dt;
    dfi(i+1)=dfi(i)+ddfi(i)*dt;
    
    l(i+1)=l(i)+dl(i)*dt;
    fi(i+1)=fi(i)+dfi(i)*dt;
     
end

plot(l)
figure()
plot(fi)
figure()
for j=1:length(t)-1
    x(j)=l(j)*sin(fi(j));
    y(j)=-l(j)*cos(fi(j));
    if mod(j,10)==0
        plot(x(j),y(j),'o');
        hold on
        axis([-5 5 -5 5])
        pause(0.001)
    end
end


