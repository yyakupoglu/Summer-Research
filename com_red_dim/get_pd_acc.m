function a = get_pd_acc(p_d, p, v_d, v, a_d)
a = a_d+500*(p_d-p)+10*(v_d-v);
end