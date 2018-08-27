clear;
close all;

addpath('utils');
addpath('trajectories');

controlhandle = @controller;

% Choose which trajectory you want to test with
trajhandle = @traj_line; 
% trajhandle = @traj_sine;
% trajhandle = @traj_step;
% trajhandle = @traj_diamond;

[t, state, d_state] = simulation_2d(controlhandle, trajhandle);
[ params ] = sys_params;
disp(['Following trajectory error: ', num2str(trajerr(state, d_state))]);
disp(['Weight of payload [kg]: ', num2str(params.payload.mass)]);

[E, Eres] = enconsum(state, trajhandle, params);
disp(['Energy consumption: ', num2str(ceil(E)), ' [J]']);
disp(['Left energy: ', num2str(ceil(Eres/1000)), ' [kJ]']);

% windplot(t,state);
