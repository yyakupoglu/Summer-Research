function a = get_pd_acc(p_d, p, v_d, v, a_d)
a = a_d+1000*(p_d-p)+100*(v_d-v);
end