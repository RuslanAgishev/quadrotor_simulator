function [err, n] = trajerr(state, des_state, t, pos_tol, vel_tol, vel, waypoints)

dt = 0.1;
k = dt*100;
n = 0;
des_state = des_state(1:size(state,1),:);
x = state(:,1);
x_des = des_state(:,1);

y = state(:,2);
y_des = des_state(:,2);

z = state(:,3);
z_des = des_state(:,3);

err = norm(y-y_des) + norm(z-z_des) + norm(x-x_des);
if nargin == 7
    r = pos_tol;
    dev = sqrt((x-x_des).^2 + (y-y_des).^2 +(z-z_des).^2);
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = (1/vel) * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    T = [0, cumsum(d0)];
    t_index = zeros(1,length(T)-1);
    for i=1:(length(T)-2)
        t_index(i) = find(t>=T(i+1),1);
    end
    t_index(end) = length(t);
    
    v = sqrt(state(:,4).^2 + state(:,5).^2 + state(:,6).^2);
    for i=1:(length(T)-2)
        if (sum(dev((t_index(i)-k):(t_index(i)+k)) < r) > 0) && (v(t_index(i)) > vel/2)
            n = n+1;
        end
    end
    if (sum(dev((t_index(i)-2*k):(t_index(i))) < r) > 0) && (v(t_index(end)) < vel_tol)
        n = n+1;
    end
    
    % Visualisation of errors
    figure(4);
%     subplot(2,1,1);
%     plot(t,abs(state(:,1:3)-des_state));
%     grid on;
%     title('Coordinate errors: x(blue), y(red), z(yellow)');
%     xlabel('t, [s]');
%     ylabel('|r_i-r_d_e_s,_i|');
%     subplot(2,1,2);
    R = zeros(1,length(t));
    for i=1:(length(T)-2)
        R((t_index(i)-k):(t_index(i)+k)) = r;
    end
    R((t_index(end)-2*k):(t_index(end))) = r;
    plot(t,dev,t,R);
    grid on;
    title('Trajectory deviation: ');
    xlabel('t, [s]');
    ylabel('|r-r_d_e_s|');
    
end
end