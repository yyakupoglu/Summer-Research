function I_head_cm=calculateLinkInertiaAtCoM(body,L,r)
I_head_cm=diag([body.Mass*(r^2)/2 body.Mass*(3*r^2+(L)^2)/12 body.Mass*(3*r^2+(L)^2)/12]);
end