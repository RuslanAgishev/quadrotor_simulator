function odom_msg = state_to_odom(x)
% x = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
odom_msg = rosmessage('nav_msgs/Odometry');
odom_msg.Header.FrameId = 'world';
odom_msg.Header.Stamp = rostime('now');

odom_msg.Pose.Pose.Position.X = x(1);
odom_msg.Pose.Pose.Position.Y = x(2);
odom_msg.Pose.Pose.Position.Z = x(3);

odom_msg.Pose.Pose.Orientation.W = x(7);
odom_msg.Pose.Pose.Orientation.X = x(8);
odom_msg.Pose.Pose.Orientation.Y = x(9);
odom_msg.Pose.Pose.Orientation.Z = x(10);

odom_msg.Twist.Twist.Linear.X = x(4);
odom_msg.Twist.Twist.Linear.Y = x(5);
odom_msg.Twist.Twist.Linear.Z = x(6);

odom_msg.Twist.Twist.Angular.X = x(11);
odom_msg.Twist.Twist.Angular.Y = x(12);
odom_msg.Twist.Twist.Angular.Z = x(13);
end