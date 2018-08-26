function I=getSystemInertia(r)
    I=[];
    for link_no = 1:r.NumBodies-1
        T=getLinkSpatialInertia(r.Bodies{1,link_no});
        I=blkdiag(I,T);
    end
    
end
