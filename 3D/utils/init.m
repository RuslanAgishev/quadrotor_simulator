function [params,maxtime,pos_tol,vel_tol,trajhandle,velocity,sigmaN] = init()
params = sys_params();
maxtime = 10; % maximum time of the simulation
pos_tol = 0.1; % [m]
vel_tol = 0.1; % [m/s]

% Trajectory generation with waypoints
trajhandle = @traj_generator;
velocity = 1.8; % desired mean velocity

sigmaN = 0.02;
end