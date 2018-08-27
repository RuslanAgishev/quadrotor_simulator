function err = trajerr(state, d_state)
y = state(:,1);
y = y(1:5:length(y));
y_des = d_state(:,1);

z = state(:,2);
z = z(1:5:length(z));
z_des = d_state(:,2);

err = norm(y-y_des) + norm(z-z_des);
% plot(abs(y-y_des), abs(z-z_des));
figure;
plot(state(:,1),state(:,2),d_state(:,1),d_state(:,2));
axis equal
grid on
xlabel('y [m]'); ylabel('z [m]');
title('Desired and real trajectories: ');
end