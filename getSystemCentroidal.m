function XS=getSystemCentroidal(r,config)
com_to_base = getCoMtransform(r.centerOfMass);
XS=[];
for link_no = 1:r.NumBodies-1
    
    base_to_link = getTransform(r,config,r.Base.Name,r.Bodies{1,link_no}.Name);
    com_to_link  = com_to_base*base_to_link;
    x=pluho(com_to_link);
    XS=[XS;x];
end
end