function [ E ] = enconsum(t, state, trajhandle, controlhandle, params)
% state - n x 13 state, with each row having format
% [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

% s.pos = [x; y; z], s.vel = [x_dot; y_dot; z_dot],
% s.rot = [phi; theta; psi], s.omega = [p; q; r]

s = cell(1,length(t));
des_s = cell(1,length(t));
R = cell(1,length(t));

F = zeros(1,length(t));
N = zeros(1,length(t));
for i=1:length(t)
    s{i} = stateToQd(state(i,:));
    des_s{i} = trajhandle(t(i),s{i});
    F(i) = controlhandle(t(i), s{i}, des_s{i}, params); % [F] = 1N
    R{i} = QuatToRot(state(i,7:10));
    N(i) = state(i,4:6)*(R{i}*[0;0;F(i)]);
end
dt = t(2)-t(1);
E = sum(abs(N))*dt;

end