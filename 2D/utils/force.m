function F = force(t, s, trajhandle)
params = sys_params;

st.pos = s(1:2);
st.rot = s(3);
st.vel = s(4:5);
st.omega = s(6);

des_st = trajhandle(t,[]);
F = controller(t, st, des_st, params); % [F] = 1N

end