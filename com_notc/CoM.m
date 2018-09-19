function r = CoM(q)
global robot m
sum = [0;0;0];
for i =1:robot.NumBodies-1
    tf = getTransform(robot,q,robot.Bodies{1,i}.Name,'base');
    v = tf*[robot.Bodies{1,i}.CenterOfMass 1]';
    sum = sum + robot.Bodies{1,i}.Mass*v(1:3);
end
r = sum/m;
end

