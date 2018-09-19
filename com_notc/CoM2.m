function r = CoM2(q)
global robot m
sum = [0;0;0];
x1=robot.Bodies{1,1}.CenterOfMass(1)*cos(q(1));
y1=robot.Bodies{1,1}.CenterOfMass(1)*sin(q(1));
x2o=robot.Bodies{1,2}.CenterOfMass(1)*cos(q(2));
y2o=robot.Bodies{1,2}.CenterOfMass(1)*sin(q(2));
x2=2*x1+x2o;
y2=2*x2+y2o;
x3=x2+x2o+robot.Bodies{1,3}.CenterOfMass(1)*cos(q(3));
y3=y2+y2o+robot.Bodies{1,3}.CenterOfMass(1)*sin(q(3));
A = [x1 x2 x3;
     y1 y2 y3;
     0  0  0];
mm = [robot.Bodies{1,1}.Mass;
      robot.Bodies{1,2}.Mass;
      robot.Bodies{1,3}.Mass];
r = A*mm/m;  
      
    
end
