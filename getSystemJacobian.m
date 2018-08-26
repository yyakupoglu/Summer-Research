function JS=getSystemJacobian(r,config)
JS=[];
for link_no = 1:r.NumBodies-1
    j = geometricJacobian(r,config,r.Bodies{1,link_no}.Name);
    JS=[JS;j];
end
end