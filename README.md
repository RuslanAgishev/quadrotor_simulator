# Quadrotor simulator

## Package description

Quadrotor simulation writen in Matlab with ROS communication.
The package is written based on wonderful course from University of Pennsylvania on [Coursera](https://www.coursera.org/lecture/robotics-flight/quadrotors-XguwZ).

<img src="https://github.com/RuslanAgishev/quadrotor_simulator/blob/master/figures/snap_traj3D.png" width=400/>
Fig. 1. Minimum snap trajectory via waypoints

## Running the simulation
Clone the repository:
```bash
git clone https://github.com/RuslanAgishev/quadrotor_simulator.git
cd quadrotor_simulator/3D
```
From your Matlab command prompt run:
```matlab
runsim.m
```
You would see a quadrotor executing a min-jerk trajectory via sequence of spiral waypoints around a pointcloud object.
<img src="https://github.com/RuslanAgishev/quadrotor_simulator/blob/ltu/figures/pointcloud_spiral.jpg" />

## ROS integration
During the fligt the drone's state is published as a ROS ```nav_msgs/Odometry``` message.
You could also access quadrotor's controller commands published as array ```std_msgs/Float32MultiArray``` message.
Connect to the master in order to see the topics:
```bash
export ROS_MASTER_URI=http://YOUR_COMPUTER_NAME_HERE:11311/
rostopic list
```
It is also possible to look at the prerecorder rosbag file from the simulation. Follow the next steps to do this.
1. Download the [bagfile](https://drive.google.com/open?id=1KsbR3y4up9cvxbdBU-nVwfyiaTgoTL7l) and place it in the
[launch](https://github.com/RuslanAgishev/quadrotor_simulator/tree/ltu/ros_ws/src/quadrotor_sim/launch) directory.

2. Build the workspace:
```bash
cd quadrotor_simulator/ros_ws
catkin_make
source devel/setup.bash
```
3. Adjust the poitcloud file path [here](https://github.com/RuslanAgishev/quadrotor_simulator/blob/ltu/ros_ws/src/quadrotor_sim/src/pc_publisher.py#L54) and play recorded data:
```bash
roslaunch quadrotor_sim quadrotor_sim.launch
```

As an output you should see in Rviz:
<img src="https://github.com/RuslanAgishev/quadrotor_simulator/blob/ltu/figures/spiral_rviz.png" />

## Implementation details
Minimum jerk and minimum snap trajectories generation implementation is available [here](https://github.com/RuslanAgishev/quadrotor_simulator/blob/ltu/3D/traj_generator.m).
Reference: [Daniel Mellinger and Vijay Kumar, 2011](http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf).

Quadrotor controller is implemented [here](https://github.com/RuslanAgishev/quadrotor_simulator/blob/ltu/3D/controller.m).
Reference: [Coursera lecture from Prof. Vijay Kumar](https://ru.coursera.org/lecture/robotics-flight/3-d-quadrotor-control-zpCD1).
