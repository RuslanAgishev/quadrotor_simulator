function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

% Coefficients for regulator
PD = true;
if PD
    % Propotional + Derivative controller
    Kpx    = 10;    Kdx    = 2;
    Kpy    = 10;    Kdy    = 2;
    Kpz    = 100;   Kdz    = 2;      % Force u1
    Kpphi  = 10;    Kdphi  = 1;      % Moment u2(1)
    Kpth   = 10;    Kdth   = 1;      % Moment u2(2)
    Kppsi  = 10;    Kdpsi  = 1;      % Moment u2(3)
    
else
    % Proportional controller, fast visualisation
    Kpx    = 10;    Kdx    = 0;
    Kpy    = 10;    Kdy    = 0;
    Kpz    = 100;   Kdz    = 0;
    Kpphi  = 10;    Kdphi  = 0;
    Kpth   = 10;    Kdth   = 0;
    Kppsi  = 10;    Kdpsi  = 0;
    
end

% Thrust u1 = F
F = params.mass*(params.gravity - Kdz*state.vel(3)-...
    Kpz*(state.pos(3)-des_state.pos(3)));

acc_c(1) = des_state.acc(1) + Kpx*(des_state.pos(1) - state.pos(1)) +...
    Kdx*(des_state.vel(1) - state.vel(1));
acc_c(2) = des_state.acc(2) + Kpy*(des_state.pos(2) - state.pos(2)) +...
    Kdy*(des_state.vel(2) - state.vel(2));

phi_c = (1/params.gravity)*(acc_c(1)*sin(des_state.yaw)-...
    acc_c(2)*cos(des_state.yaw));
th_c = (1/params.gravity)*(acc_c(1)*cos(des_state.yaw)+...
    acc_c(2)*sin(des_state.yaw));

p_des = 0; q_des = 0; r_des = des_state.yawdot;

% Moment u2 = M
M = zeros(3,1);
M(1) = Kpphi*(phi_c-state.rot(1)) + Kdphi*(p_des-state.omega(1));
M(2) = Kpth*(th_c-state.rot(2)) + Kdth*(q_des-state.omega(2));
M(3) = Kppsi*(des_state.yaw-state.rot(3)) + Kdpsi*(r_des-state.omega(3));

end
