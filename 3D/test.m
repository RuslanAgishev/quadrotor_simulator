close all;
clear;
addpath('utils');

[params,maxtime,pos_tol,vel_tol,trajhandle,velocity,sigmaN] = init();

random = true;
if random
    N = 4;  % whole number of random waypoints
    d = 3;  % maximum value of each coordinate (x,y,z)    waypoints = [zeros(3,1) randi(d,3,N)];
else
    form = 'maneuvre';
    waypoints = getwp(form, 4, pi/13);
    % waypoints = getwp(form);
    
    %         waypoints = [0    0   0;
    %                      1    1   1;
    %                      2    0   2;
    %                      3    -1  1;
    %                      4    0   0]';
    
    N = length(waypoints)-1;
end
% [mode, minangle] = traj_type(waypoints);
mode = 'acc';
trajhandle([], [], mode, velocity, waypoints);
% controller
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
% s - n x 13 state, with each row having format
% [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

disp(['Trajectory type: ', mode]);

[t, state, des_state] = simulation_3d(trajhandle, controlhandle,...
    maxtime, pos_tol, vel_tol, waypoints);
[err, n] = trajerr(state, des_state,t, pos_tol, vel_tol, velocity, waypoints);
disp(['Waypoints passed successfully: ', num2str(n), '/', num2str(N)]);

% filtration
[xm,ym,zm] = measurements(state, 0.01);
Z = [xm ym]';
T = 1;
sigmaA = 0.01;

X0 = zeros(4,1);
x1 = X0(1);
y1 = X0(3);
P0 = 10^5*eye(4);

[F,G,H] = state_space(T);
Q = G*G'*sigmaA^2;
R = [sigmaN^2 0; 0 sigmaN^2];

[~,~,Xfl,Pfl,~,~] = kalman_filter(X0,P0,F,H,R,Q,Z, sigmaN);

xfl = Xfl(1,:);
yfl = Xfl(3,:);

x = state(:,1);
y = state(:,2);

figure
plot(x,y, xm,ym,':', xfl,yfl)
grid on
title('Cartesian coordinates')
xlabel('x')
ylabel('y')
legend('real', 'measurements', 'filtered')


