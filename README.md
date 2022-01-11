To run the Gazebo simulator:

```sh
# first make a catkin workspace if you haven't done so.
source /opt/ros/noetic/setup.bash   # if you haven't done so.
mkdir -p ./ros_ws/src
cd ./ros_ws/src
git clone https://github.com/HaoguangYang/omniveyor_gazebo_world.git
cd ..
# compile the package.
catkin_make
```

Fix any dependency issues should they causes compilation error. Then:

```sh
source ./devel/setup.sh
roslaunch omniveyor_gazebo_world IMI.launch
```

You should see three robots spawned in the simulated 3D factory floor.

Current status:

- The differential-drive robot is set up for manually specifying joint commands in the Gazebo GUI.
- The treaded robot is set up to receive Twist command from ROS. Try:
```sh
rostopic pub /simple_tracked/cmd_vel_twist geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
- The OmniVeyor is set up with a full suite of LiDAR and camera sensors.

Work In Progress:

- A plugin that publishes the name of object selected in Gazebo GUI.
- A control interface for the OmniVeyors. (currently only manual joint speed setting through Gazebo GUI is allowed)
- Integration with the task dispatcher of the MobileManipulation software stack.
- Kind-of slow (sometimes Real Time ratio ~ 0.45), still figuring out why.
