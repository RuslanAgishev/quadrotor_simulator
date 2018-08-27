% WIND returns the 2D-force of wind W acting on the surface of the
% quadrotor. I.e. W(1) = Wy is caused by the existance of the Sz projection
% of the surface of the system S on Oz-axis.
function [W,vel,ang] = wind(t, state)
    params = sys_params;
%     vel = params.wind.vel + 2*sin(10*t);
% 
%     ang = params.wind.ang * ((t<=1) + ((1<t)&&(t<=2))*11/5 +...
%                             ((2<t)&&(t<=3))*12/8 + ((3<t)&&(t<=4))*3/12);
    vel = params.wind.vel;
    ang = params.wind.ang;

    W = zeros(1,2);
    W(1) = sign(cos(ang))*params.cd*params.air_dens*(state.vel(1)-vel*cos(ang))^2*...
        (abs(params.cross.square * sin(state.rot(1))) + params.payload.square)/2;
    W(2) = sign(sin(ang))*params.cd*params.air_dens*(state.vel(2)-vel*sin(ang))^2*...
        (abs(params.cross.square * cos(state.rot(1))) + params.payload.square)/2;
end