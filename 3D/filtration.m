% FILTRATION
close all;
addpath('utils');
addpath('real_flight');

% Data from real flight
outdoor_flight_process;
state = zeros(length(x),3);
state(:,1) = x;
state(:,2) = y;
state(:,3) = z;
t = 0.05*(1:length(x));

%runsim;
sigmaN = 5;
[xm,ym,zm] = measurements(state, sigmaN);
Z = [xm ym zm]';
T = abs(t(2)-t(1));

sigmaA = 4;

X0 = zeros(6,1);
x1 = X0(1);
y1 = X0(3);
z1 = X0(5);
P0 = 10^5*eye(6);

[F,G,H] = state_space(T);
Q = G*G'*sigmaA^2;
R = [sigmaN^2 0 0; 0 sigmaN^2 0; 0 0 sigmaN^2];

[~,~,Xfl,Pfl,~,V,~] = kalman_filter(X0,P0,F,H,R,Q,Z, sigmaN);

xfl = Xfl(1,:);
yfl = Xfl(3,:);
zfl = Xfl(5,:);

x = state(:,1);
y = state(:,2);
z = state(:,3);

vx = V(1,:);
vy = V(2,:);
vz = V(3,:);

M = 13;
vx_sm = runningmean(vx,M);
vy_sm = runningmean(vy,M);
vz_sm = runningmean(vz,M);

figure
plot(x,y, xm,ym,':', xfl,yfl)
grid on
title('Cartesian coordinates')
xlabel('x, [m]')
ylabel('y, [m]')
legend('real', 'measurements', 'filtered')

figure
subplot(1,2,1)
title('Residual Vx = zx - xm')
plot(vx,':')
hold on
plot(vx_sm)
grid on
legend('measured','smoothed')
ylabel('Vx, [m]')
xlabel('time')

subplot(1,2,2)
title('Residual Vy = zy - ym')
plot(vy,':')
hold on
plot(vy_sm)
legend('measured','smoothed')
grid on
ylabel('Vy, [m]')
xlabel('time')

% determine suitable sigmaN-scale
figure
plot(abs(vy_sm))
hold on
plot(2.5*sigmaN*ones(1,length(vy_sm)))
plot(2*sigmaN*ones(1,length(vy_sm)))
plot(1*sigmaN*ones(1,length(vy_sm)))
plot(abs(vx_sm))
grid on
legend('|zy-ym|', '2.5 sigma_N','2.0 sigma_N', '1.0 sigma_N','|zx-xm|')
xlabel('time')
ylabel('residual, [m]')

scale = 2;
[~,~,Xfl_imp,Pfl_imp,~,V_imp,cndt] = kalman_filter(X0,P0,F,H,R,Q,Z, sigmaN,scale,[vx_sm; vy_sm; vz_sm]);

xfl_imp = Xfl_imp(1,:);
yfl_imp = Xfl_imp(3,:);
zfl_imp = Xfl_imp(5,:);

figure
plot(x,y, xm,ym,'o', xfl_imp,yfl_imp, xfl,yfl)
grid on
title('Improved filtration')
xlabel('x, [m]')
ylabel('y, [m]')
legend('real','measure','improved','filtered')

figure
% subplot(1,2,1)
title('X-coordinate')
plot(xm,':')
hold on
plot(xfl_imp)
hold on
plot(xfl)
hold on
%plot(max([cndt(1,:),cndt(2,:),cndt(3,:)]))
grid on
legend('measure','improved', 'filtered')
xlabel('time')
ylabel('x, [m]')

figure
% subplot(1,2,2)
title('Y-coordinate')
plot(ym,':')
hold on
plot(yfl_imp)
hold on
plot(yfl)
hold on
%plot(max([cndt(1,:),cndt(2,:),cndt(3,:)]))
grid on
legend('measure','improved', 'filtered')
xlabel('time')
ylabel('y, [m]')

figure
% subplot(1,3,3)
title('Z-coordinate')
plot(zm,':')
hold on
plot(zfl_imp)
hold on
plot(zfl)
hold on
%plot(max([cndt(1,:),cndt(2,:),cndt(3,:)]))
grid on
legend('measure','improved', 'filtered')
xlabel('time')
ylabel('z, [m]')

figure
mses = categorical({'MSE(x,xfl)','MSE(x,xfl_imp)';'MSE(y,yfl)',...
    'MSE(y,yfl_imp)';'MSE(z,zfl)','MSE(z,zfl_imp)'});
bar(mses,[immse(x,xfl'),immse(x,xfl_imp'); immse(y,yfl'),immse(y,yfl_imp');...
    immse(z,zfl'),immse(z,zfl_imp')], 2.0)
title('MSE: KF vs IKF')
ylabel('MSE')
legend('KF', 'IKF')

figure
plot3(x,y,z)
grid on
%hold on
%plot3(xfl_imp,yfl_imp,zfl_imp)
%hold on
%plot3(xm,ym,zm, '.')
%hold on
%plot3(xfl,yfl,zfl)
xlabel('X, [m]')
ylabel('Y, [m]')
zlabel('Z, [m]')
title('3D-trajectory')
legend('real','IKF','measurements','KF')

% fprintf(strcat('MSE(x,xfl)=',num2str(immse(x,xfl')),'\n'))
% fprintf(strcat('MSE(x,xfl_imp)=',num2str(immse(x,xfl_imp')),'\n'))
% fprintf(strcat('MSE(y,yfl)=',num2str(immse(y,yfl')),'\n'))
% fprintf(strcat('MSE(y,yfl_imp)=',num2str(immse(y,yfl_imp')),'\n'))
