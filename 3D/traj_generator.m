function [ desired_state ] = traj_generator(t, state, mode, vel, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.

persistent w0 S d0 m
if nargin > 2
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 1/vel * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        S = [0, cumsum(d0)];
        w0 = waypoints;
        m = mode;
%% Stright line trajectory:
% This is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s.
else
    if strcmp(m,'line')
        
        if(t > S(end))
            t = S(end);
        end
        t_index = find(S >= t,1);
        
        if(t_index > 1)
            t = t - S(t_index-1);
        end
        if(t == 0)
            desired_state.pos = w0(:,1);
        else
            scale = t/d0(t_index-1);
            desired_state.pos = (1 - scale) * w0(:,t_index-1) + scale * w0(:,t_index);
        end
        
        desired_state.vel = 0.5*ones(3,1);
        desired_state.acc = zeros(3,1);
        
        if numel(state)~=0
            desired_state.yaw = state.rot(3);
            desired_state.yawdot = state.omega(3);
        else
            desired_state.yaw = 0;
            desired_state.yawdot = 0;
        end
    
%% Mininum snap trajectory
    elseif strcmp(m, 'snap')
        % creating matrices A and b in Aa=b
        n = size(w0,2)-1;
        A = zeros(8*n,8*n);
        b = zeros(8*n,3);
        
        for i=1:n
            % pi(S(i-1)) = w(i-1)
            A(i,(1+8*(i-1)):(8*i)) = give_matrix(S(i),S(i),d0(i), 1, m);
            b(i,:) = w0(:,i)';
            % pi(S(i)) = w(i)
            A(i+n,(1+8*(i-1)):(8*i)) = give_matrix(S(i+1),S(i),d0(i), 1, m);
            b(i+n,:) = w0(:,i+1)';
        end
        % p1(k)(S0) = 0
        A(1+2*n,1:8) = give_matrix(0,0,d0(1), 2, m); % k=1
        A(2+2*n,1:8) = give_matrix(0,0,d0(1), 3, m); % k=2
        A(3+2*n,1:8) = give_matrix(0,0,d0(1), 4, m); % k=3
        
        % pn(k)(Sn) = 0
        A(4+2*n,(8*n-7):(8*n)) = give_matrix(S(n+1),S(n),d0(n), 2, m); % k=1
        A(5+2*n,(8*n-7):(8*n)) = give_matrix(S(n+1),S(n),d0(n), 3, m); % k=2
        A(6+2*n,(8*n-7):(8*n)) = give_matrix(S(n+1),S(n),d0(n), 4, m); % k=3
        
        % pi(k)(Si)-p(i+1)(k)(Si) = 0
        for k=1:6
            for i=1:(n-1)
                A(i+(k+1)*n+6 - (k-1),(1+8*(i-1)):(16+8*(i-1))) =...
                    [give_matrix(S(i+1),S(i),d0(i), k+1, m),-give_matrix(S(i+1),S(i+1),d0(i+1), k+1, m)]; % k=1:6
            end
        end
        
        % a = [x(t), y(t), z(t)]
        a = A\b;
        
        if(t > S(end))
            t = S(end);
        end
        t_index = find(S >= t,1);
        
        if(t_index > 1)
            t = t - S(t_index-1);
        end
        if(t_index == 1)
            desired_state.pos = w0(:,1);
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
            desired_state.yaw = 0;
            desired_state.yawdot = 0;
        else
            desired_state.pos = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 1, m)...
                *a((1+8*(t_index-2)):(8*(t_index-1)),:))';
            desired_state.vel = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 2, m)...
                *a((1+8*(t_index-2)):(8*(t_index-1)),:))';
            desired_state.acc = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 3, m)...
                *a((1+8*(t_index-2)):(8*(t_index-1)),:))';
            if numel(state)~=0
                desired_state.yaw = state.rot(3);
                desired_state.yawdot = state.omega(3);
            else
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
        
%% Mininum jerk trajectory
    elseif strcmp(m, 'jerk')
        % creating matrices A and b in Aa=b
        n = size(w0,2)-1;
        A = zeros(6*n,6*n);
        b = zeros(6*n,3);
        
        for i=1:n
            % pi(S(i-1)) = w(i-1)
            A(i,(1+6*(i-1)):(6*i)) = give_matrix(S(i),S(i),d0(i), 1, m);
            b(i,:) = w0(:,i)';
            % pi(S(i)) = w(i)
            A(i+n,(1+6*(i-1)):(6*i)) = give_matrix(S(i+1),S(i),d0(i), 1, m);
            b(i+n,:) = w0(:,i+1)';
        end
        % p1(k)(S0) = 0
        A(1+2*n,1:6) = give_matrix(0,0,d0(1), 2, m); % k=1
        A(2+2*n,1:6) = give_matrix(0,0,d0(1), 3, m); % k=2
        
        % pn(k)(Sn) = 0
        A(3+2*n,(6*n-5):(6*n)) = give_matrix(S(n+1),S(n),d0(n), 2, m); % k=1
        A(4+2*n,(6*n-5):(6*n)) = give_matrix(S(n+1),S(n),d0(n), 3, m); % k=2
        
        % pi(k)(Si)-p(i+1)(k)(Si) = 0
        for k=1:4
            for i=1:(n-1)
                A(i+(k+1)*n+4 - (k-1),(1+6*(i-1)):(12+6*(i-1))) =...
                    [give_matrix(S(i+1),S(i),d0(i), k+1, m),-give_matrix(S(i+1),S(i+1),d0(i+1), k+1, m)]; % k=1:4
            end
        end
        
        % a = [x(t), y(t), z(t)]
        a = A\b;
        
        if(t > S(end))
            t = S(end);
        end
        t_index = find(S >= t,1);
        
        if(t_index > 1)
            t = t - S(t_index-1);
        end
        if(t_index == 1)
            desired_state.pos = w0(:,1);
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
            desired_state.yaw = 0;
            desired_state.yawdot = 0;
        else
            desired_state.pos = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 1, m)...
                *a((1+6*(t_index-2)):(6*(t_index-1)),:))';
            desired_state.vel = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 2, m)...
                *a((1+6*(t_index-2)):(6*(t_index-1)),:))';
            desired_state.acc = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 3, m)...
                *a((1+6*(t_index-2)):(6*(t_index-1)),:))';
            if numel(state)~=0
                desired_state.yaw = state.rot(3);
                desired_state.yawdot = state.omega(3);
            else
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
        
%% Mininum acceleration trajectory
    elseif strcmp(m, 'acc')
        % creating matrices A and b in Aa=b
        n = size(w0,2)-1;
        A = zeros(4*n,4*n);
        b = zeros(4*n,3);
        
        for i=1:n
            % pi(S(i-1)) = w(i-1)
            A(i,(1+4*(i-1)):(4*i)) = give_matrix(S(i),S(i),d0(i), 1, m);
            b(i,:) = w0(:,i)';
            % pi(S(i)) = w(i)
            A(i+n,(1+4*(i-1)):(4*i)) = give_matrix(S(i+1),S(i),d0(i), 1, m);
            b(i+n,:) = w0(:,i+1)';
        end
        % p1(k)(S0) = 0
        A(1+2*n,1:4) = give_matrix(0,0,d0(1), 2, m); % k=1
        
        % pn(k)(Sn) = 0
        A(2+2*n,(4*n-3):(4*n)) = give_matrix(S(n+1),S(n),d0(n), 2, m); % k=1
        
        % pi(k)(Si)-p(i+1)(k)(Si) = 0
        for k=1:2
            for i=1:(n-1)
                A(i+(k+1)*n+2 - (k-1),(1+4*(i-1)):(8+4*(i-1))) =...
                    [give_matrix(S(i+1),S(i),d0(i), k+1, m),-give_matrix(S(i+1),S(i+1),d0(i+1), k+1, m)]; % k=1:4
            end
        end
        
        % a = [x(t), y(t), z(t)]
        a = A\b;
        
        if(t > S(end))
            t = S(end);
        end
        t_index = find(S >= t,1);
        
        if(t_index > 1)
            t = t - S(t_index-1);
        end
        if(t_index == 1)
            desired_state.pos = w0(:,1);
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
            desired_state.yaw = 0;
            desired_state.yawdot = 0;
        else
            desired_state.pos = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 1, m)...
                *a((1+4*(t_index-2)):(4*(t_index-1)),:))';
            desired_state.vel = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 2, m)...
                *a((1+4*(t_index-2)):(4*(t_index-1)),:))';
            desired_state.acc = (give_matrix(t+S(t_index-1), S(t_index-1), d0(t_index-1), 3, m)...
                *a((1+4*(t_index-2)):(4*(t_index-1)),:))';
            if numel(state)~=0
                desired_state.yaw = state.rot(3);
                desired_state.yawdot = state.omega(3);
            else
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
    end
end
end

