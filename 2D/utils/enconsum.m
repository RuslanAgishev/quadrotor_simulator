function [E, Eres] = enconsum(state, trajhandle, params)
% E = int(Rds) from A(y0,z0) to B(y,z)
% m = params.mass;
% g = params.gravity;

phi = state(:,3);
vy = state(:,4);
vz = state(:,5);

N = length(state);
% getting the thrust and wind forces for each time moment
% W = windplot(t,state);
F = nan(N,1);
dt = 0.01;
for i = 1:N
    F(i) = force(i*dt, state(i,:), trajhandle);
end
% computing the whole work required for the trajectory following

Fy = -(F.*sin(phi))';
Fz = (F.*cos(phi))';
E = (Fy*vy + Fz*vz) * dt * N; % [E] = 1J = 1 kg*m^2/s^2
Eres = params.quad.energy * 4 - E;
if Eres < 0, disp('More powerful bataries are required\n');
end;