function windplot(t,state)
% WINDPLOT visualises wind force in both (y-, z-) diections
N = length(t);
W = zeros(N,2);
vel = zeros(1,N);
ang = zeros(1,N);
for i=1:N
    st.pos = state(i,1:2);
    st.rot = state(i,3);
    st.vel = state(i,4:5);
    st.omega = state(i,6);
    [V,vel(i),ang(i)] = wind(t(i),st);
    W(i,1) = V(1);
    W(i,2) = V(2);
end
figure(1);
subplot(1,2,1);
plot(t,W(:,1));
grid on;
xlabel('t [s]'); ylabel('Wy [N]');
title('Horizontal wind force');
subplot(1,2,2);
plot(t,W(:,2));
grid on;
xlabel('t [s]'); ylabel('Wz [N]');
title('Vertical wind force');

figure(2);
subplot(1,2,1);
plot(t,vel);
grid on;
xlabel('t [s]'); ylabel('vel [m/s]');
title('Speed of wind');
subplot(1,2,2);
plot(t,ang);
grid on;
xlabel('t [s]'); ylabel('angle [rad]');
title('Direction of wind');
end