function tf= getCoMtransform(vec)
tf=[ eye(3) vec;
      zeros(1,3)    1];
end