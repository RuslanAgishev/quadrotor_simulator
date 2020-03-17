close all;
clear;


addpath('utils');
% initialize ROS master
rosshutdown
rosinit

params = sys_params();
maxtime = 500; % maximum time of the simulation
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
        
    % sprral waypoints
    waypoints =getwp('helix');
%     waypoints = [0    0   0;
%                      1    1   1;
%                      2    0   2;
%                      3    -1  1;
%                      4    0   0]';

    N = length(waypoints)-1;
    
    mode = 'jerk';
    trajhandle([], [], mode, velocity, waypoints);
    disp(['Trajectory type: ', mode]);
end

% CONTROLLER
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
% s - n x 13 state, with each row having format
% [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

if isequal(trajhandle, @traj_generator)
    [t, state, des_state] = simulation_3d(trajhandle, controlhandle, maxtime, pos_tol, vel_tol, waypoints);
    [err, n] = trajerr(state, des_state,t, pos_tol, vel_tol, velocity, waypoints);
    disp(['Waypoints passed successfully: ', num2str(n), '/', num2str(N)]);
else
    [t, state, des_state] = simulation_3d(trajhandle, controlhandle, maxtime, pos_tol, vel_tol);
    [ err ] = trajerr(state, des_state,t);
end

