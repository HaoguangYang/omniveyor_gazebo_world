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
rostopic pub /simple_tracked/cmd_vel_twist geometry_msgs/Twist
"linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
- The OmniVeyor is set up with a full suite of LiDAR and camera sensors. Its motion is a testing Work In Progress. To control its motion and receive its wheel odometry, try:
```sh
rostopic pub /omniveyor_robot/mobile_base_controller/cmd_vel geometry_msgs/Twist 
"linear:
  x: 1.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0" 
```
and
```sh
rostopic echo /omniveyor_robot/odom
```

Troubleshooting:
- If LiDAR returns all inf readings, your computer probably don't have a capable GPU to run GPU ray tracing. Change `models/Omniveyor/model.sdf` Lines 423 and 440 by removing the `gpu` strings, i.e.:
```xml
<sensor name="laser" type="ray">
```
and
```xml
<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
```
- If the robot initialized "Flying off", the solver failed to converge. Restart the simulation.

Work In Progress:

- A plugin that publishes the name of object selected in Gazebo GUI.
- Integration with the task dispatcher of the MobileManipulation software stack.
- Kind-of slow (sometimes Real Time ratio ~ 0.45), still figuring out why.
