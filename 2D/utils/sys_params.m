function [ params ] = sys_params()
% sys_params System parameters

% parameters of a single quadrotor DJI F450 + E600
params.quad.mass       = 1.691; % 0.970 + 0.721 [kg] = (frame+pixhawk+propulsion)+batery
params.quad.thrustmax  = 6.400; % 4*1.600 [kg]
params.quad.k          = params.quad.thrustmax/params.quad.mass; % thrust/weigth coefficient u1 = k*mg
% E600 batery
params.quad.charge     = 4500;  % [mA*h]
params.quad.voltage    = 22.2;  % [V]
params.quad.energy     = 1/2 * params.quad.charge * params.quad.voltage * 3.6; % [J]

% parameters of a ball-payload
params.payload.mass    = 15;  % [kg]
params.payload.radius  = 0.1; % [m]
params.payload.Ixx     = 2/5*params.payload.mass*params.payload.radius^2;
params.payload.square  = pi*params.payload.radius^2; % [m^2] - square of ball-section (Sy = Sz = pi*R^2)

% parameters of a cube-payload
% params.payload.mass  = 5;  % [kg]
% params.payload.side  = 0.2; % [m]
% params.payload.Ixx   = 1/6*params.payload.mass*params.payload.side^2;

% parameters of a system "cross + payload + 4*quads"
params.cross.length = 0.5;    % distance [m] between middle of the cross and quadrotor
params.cross.width  = 0.1;    % width [m] of plank, cross is two planks
params.cross.mass   = 2*0.5;  % mass [kg] of a cross
params.cross.square = 2*2*params.cross.length*params.cross.width; % square [m^2] of a cross
params.mass         = params.cross.mass + 4*params.quad.mass + params.payload.mass;
params.Ixx          = 2*params.quad.mass*params.cross.length^2 +...
                      2*1/12*(params.cross.mass/2)*(2*params.cross.length)^2 +...
                      params.payload.Ixx;

% environmental properties
params.cd       = 1.95;
params.wind.vel = 15;     % [m/s]
params.wind.ang = pi;    % [rad]
params.air_dens = 1.225; % [kg/m^3]
% params.air_dens = 0;   % switch off the wind and air resistance
params.gravity  = 9.81;

% accuracy at the destination point
params.postol = 0.01;
params.veltol = 0.05;
params.angtol = 0.03;

params.timelim = 5; % boundary on the time of motion along the trajectory

% saturation of a thrust force of a system
params.minF = 0;
params.maxF = 4*params.quad.thrustmax * params.gravity;

% maximum weight of PAYload [kg] quadrotors can carry
params.weightmax = 4*params.quad.thrustmax - 4*params.quad.mass - params.cross.mass;

% total mass of the system [kg]
params.mass = params.cross.mass + 4*params.quad.mass + params.payload.mass;

end