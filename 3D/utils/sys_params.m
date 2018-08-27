function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor
s = 1; % scale of dimentions of the quadrotor

M = 0.1 * (s^3); % kg
m = 0.02 * (s^3); % kg
m_total = M + 4*m;
R = 0.01 * s;
r = 0.005 * s;
l = 0.086 * s;

g = 9.81; % m/s/s
params.air_dens = 1.225; % kg/m^3
params.cd = 1.95;
% m_total = 0.18; % kg
% I = [0.00025,   0,          2.55e-6;
%      0,         0.000232,   0;
%      2.55e-6,   0,          0.0003738];

I = inertia(M,m,R,l);
params.radius = R;
params.motor = r;
params.torque = 0.05; % for visualisation of rotation

params.mass = m_total;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = l; % m
params.Sx    = 0.002; % m^2
params.Sy    = 0.002;
params.Sz    = 0.004;

params.minF = 0.0;
params.maxF = 2.0*m_total*g;

end
