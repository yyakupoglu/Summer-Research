function I_i = getLinkSpatialInertia(b)
I_i=[convertInertiaVectortoMatrix(b.Inertia) b.Mass*skew(b.CenterOfMass);
     b.Mass*skew(b.CenterOfMass)'              b.Mass*eye(3)];
 
end