### To run the Gazebo simulator:

```sh
# first make a catkin workspace if you haven't done so.
source /opt/ros/noetic/setup.bash   # if you haven't done so.
mkdir -p ./ros_ws/src
cd ./ros_ws
rosinstall ./src /opt/ros/noetic https://raw.githubusercontent.com/HaoguangYang/omniveyor_gazebo_world/master/omniveyor_gazebo_world.rosinstall
# compile the package.
catkin_make -DCMAKE_BUILD_TYPE=Release
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
- The OmniVeyor is set up with a full suite of LiDAR and camera sensors. To control its motion and receive its wheel odometry, try:
```sh
rostopic pub /cmd_vel geometry_msgs/Twist 
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
rostopic echo /odom
```
If multiple OmniVeyors are spawned, the topics are encapsulated under the robot's namespace.
- To fetch LiDAR scans, refer to the `/scan` topic.
- To fetch camera RGB and depth images, refer to the `/cam_d1/depth/image_rect_raw`, `/cam_d1/color/image_raw` topics, etc.
- Referring to the launch file and urdf/xacro files in dependent package `omniveyor_common` for detailed explanation on available options.

### Troubleshooting:

- If LiDAR returns all inf readings, your computer probably don't have a capable GPU to run GPU ray tracing. Change `models/Omniveyor/model.sdf` Lines 423 and 440 by removing the `gpu` strings, i.e.:
```xml
<sensor name="laser" type="ray">
```
and
```xml
<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
```
- If the scene is dark, your GPU does not support ray tracing. Try to tune the environmental lights in the world file ( `worlds/IMI-World.world` , for example), under the `<scene>` tag.
- If the robot initialized "Flying off", the solver failed to converge. Restart the simulation.

### Work In Progress:

- A plugin that publishes the name of object selected in Gazebo GUI.
- Integration with the task dispatcher of the MobileManipulation software stack.
