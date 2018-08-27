close all;
clear;


addpath('utils');

params = sys_params();
maxtime = 10; % maximum time of the simulation
pos_tol = 0.1; % [m]
vel_tol = 0.1; % [m/s]

sample = false;
if sample
    % PRE-CALCULATED TRAJECTORIES
    % trajhandle = @traj_line;
    trajhandle = @traj_helix;
    
else
    % TRAJECTORY GENERATION WITH WAYPOINTS
    trajhandle = @traj_generator;
    velocity = 1.0; % desired mean velocity
    random = false;
    
    if random
        N = 4;  % whole number of random waypoints
        d = 3;  % maximum value of each coordinate (x,y,z)
        waypoints = [zeros(3,1) randi(d,3,N)];
    else
        form = 'maneuvre';
        %waypoints = getwp(form, 4, pi/13);
        % waypoints = getwp(form);
        
        waypoints = [0    0   0;
                     1    1   1;
                     2    0   2;
                     3    -1  1;
                     4    0   0]';

        N = length(waypoints)-1;
    end
    % [mode, minangle] = traj_type(waypoints);
    mode = 'snap';
    trajhandle([], [], mode, velocity, waypoints);
end

% CONTROLLER
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
% s - n x 13 state, with each row having format
% [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

disp(['Trajectory type: ', mode]);

if isequal(trajhandle, @traj_generator)
    [t, state, des_state] = simulation_3d(trajhandle, controlhandle, maxtime, pos_tol, vel_tol, waypoints);
    [err, n] = trajerr(state, des_state,t, pos_tol, vel_tol, velocity, waypoints);
    disp(['Waypoints passed successfully: ', num2str(n), '/', num2str(N)]);
else
    [t, state, des_state] = simulation_3d(trajhandle, controlhandle, maxtime, pos_tol, vel_tol);
    [ err ] = trajerr(state, des_state,t);
end

disp(['Trajectory error: ', num2str(err), ' [m]']);
L = length_traj(state(:,1:3));
Vmean = mean_velocity(state(:,4:6));
disp(['Trajectory length: ', num2str(L), ' [m]']);
disp(['Mean velocity: ', num2str(Vmean), ' [m/s]']);

E = enconsum(t, state, trajhandle, controlhandle, params);
disp(['Energy consumption: ', num2str(E), ' [J]']);
