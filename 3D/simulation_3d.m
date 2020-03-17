function [t_out, s_out, d_state, term_crit] = simulation_3d(trajhandle, controlhandle, max_time, pos_tol, vel_tol, waypoints)
% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** QUADROTOR SIMULATION *****************

addpath('utils');

video = false;

% real-time
real_time = false;

% parameters for simulation
params = sys_params;


%% **************************** FIGURES *****************************
disp('Initializing figures...');

if video
    video_writer = VideoWriter('traj_3d.avi', 'Uncompressed AVI');
    open(video_writer);
end

h_fig = figure;
h_3d = gca;
if nargin == 6
    r = pos_tol;
    w = waypoints;
    [a,b,c] = sphere;
    
    axis equal
    hold on
    for i=2:length(w)
        surf(r*a+w(1,i),r*b+w(2,i),r*c+w(3,i)) % centered at (w(1,i),w(2,i),w(3,i))
    end
end
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

pc = pcread('../data/fountain.ply');
pcshow(pc);

set(gcf,'Renderer','OpenGL')


%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;

d_state = nan(max_iter*nstep,3);
for iter = 1:(max_iter*nstep)
    dd = trajhandle(tstep*iter,[]);
    d_state(iter,:) = dd.pos';
end

x0    = init_state(des_start.pos, 0);
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);

x       = x0;        % state
odom_msg = state_to_odom(x); % odometry ROS msg to publish
odom_pub = rospublisher('/odom', 'nav_msgs/Odometry');

array_msg = rosmessage('std_msgs/Float32MultiArray');
commands_pub = rospublisher('/commands_array', 'std_msgs/Float32MultiArray');
% traj_msg = rosmessage('nav_msgs/Path');
% traj_pub = rospublisher('/trajectory', 'nav_msgs/Path');

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');
% Main loop
for iter = 1:max_iter
    
    timeint = time:tstep:time+cstep;
    
    tic;
    
    % Initialize quad plot
    if iter == 1
        QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
        current_state = stateToQd(x);
        desired_state = trajhandle(time, current_state);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end
    
    % Run simulation
    [F,M] = controller(iter, current_state, desired_state, params);
    % Send control commands as ROS msg
    array_msg.Data = [M(1), M(2), M(3), F];
    send(commands_pub, array_msg);
    
    
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
    x    = xsave(end, :)';
    % Publish drone state as Odometry msg
    odom_msg = state_to_odom(x);
    send(odom_pub, odom_msg);
    % traj_msg.Header = odom_msg.Header;
    % append(traj_msg.Poses.Pose, odom_msg.Pose)
    % send(traj_pub, traj_msg);
    
    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    % Update quad plot
    current_state = stateToQd(x);
    desired_state = trajhandle(time + cstep, current_state);
    QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    
    time = time + cstep; % Update simulation time
    
    if video
        writeVideo(video_writer, getframe(h_fig));
    end
    
    t = toc;
    % Check to make sure ode45 is not timing out
    %     if (t> cstep*50)
    %         err = 'Ode45 Unstable';
    %         break;
    %     end
    
    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end
    
    % Check termination criteria
    term_crit = terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time);
    if term_crit
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);

% Truncate saved variables
QP.TruncateHist();

% Plot position
h_pos = figure('Name', 'Quad position');
plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% Plot velocity
h_vel = figure('Name', 'Quad velocity');
plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

disp('Finished.')

if video
    close(video_writer);
end

t_out = ttraj;
s_out = xtraj;

end
