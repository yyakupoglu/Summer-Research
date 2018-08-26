function I_head=calculateLinkInertiaAtFrame(body,L,r)

    I_head_cm = calculateLinkInertiaAtCoM(body, L, r);
    S=skew(body.CenterOfMass);
    I_head=I_head_cm+body.Mass*S*S';

end