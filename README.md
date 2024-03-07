### Before You Start:
  This is a sub-unit for Gazebo-based simulation, part of the `omniveyor` meta-package. For working with a physical robot (**NOT** for MFET442/642 class series), we strongly recommended to follow the standard (workstation/robot) setup procedure described in the [OmniVeyor meta-package](https://github.com/HaoguangYang/omniveyor) and configuring the workstation as `Option 3`. The option enables the best consistency with the physical robots in the simulation. This package also serves as a stand-alone teaching material for the [MFET 642 ROS programming class](https://web.ics.purdue.edu/~rvoyles/Classes/ROS_MFET642/index.html), for the students to gain knowledge of the Gazebo simulator.

### To run the Gazebo simulator:
  > N.B. If you are setting up a workstation for a physical robot, and have followed the [standard setup procedure](https://github.com/HaoguangYang/omniveyor), the simulator package should be already compiled. Please go directly to step 2. **Note that the workstation setup procedure DOES NOT apply to MFET442/642 students**. For students taking MFET442 please ignore this N.B. block and continue with step 1.

  - If you are running this simulation setup as a stand-alone package, you have to compile first:
    
    ```sh
    # first make a catkin workspace if you haven't done so.
    source /opt/ros/noetic/setup.bash   # if you haven't done so.
    mkdir -p ./ros_ws/src
    cd ./ros_ws
    rosinstall ./src /opt/ros/noetic https://raw.githubusercontent.com/HaoguangYang/omniveyor_gazebo_world/master/omniveyor_gazebo_world.rosinstall
    # compile the package.
    catkin_make -DCMAKE_BUILD_TYPE=Release
    ```
    Fix any dependency issues should they causes compilation error. Most likely you wil see an error about missing `costmap_2d` package. Install it with `apt-get`:
    ```sh
    sudo apt-get install ros-noetic-costmap-2d
    ```
    Recompile the workspace and make sure the error is gone.

  - To run the simulation scene:
    ```sh
    source ./devel/setup.sh
    roslaunch omniveyor_gazebo_world IMI.launch
    ```
    You should see three robots spawned in the simulated 3D factory floor.

  - You can inspect the contents of `launch/IMI.launch` as an example of how to disable some sensors on the robot model to relief computational load, or spawn a robot at a specified coordinate. e.g. to disable cameras on the robot model, you can add arguments to the included robot launch script, for example:
    ```xml
    <include file="$(find omniveyor_gazebo_world)/launch/singleRobot.launch">
      <arg name="has_front_d435i" value="false"/>
      <arg name="has_rear_d435i" value="false"/>
      <arg name="has_t265" value="false"/>
    </include>
    ```
    To control where the robot is spawned, you can add arguments specifying its initial coordinates, like:
    ```xml
    <include file="$(find omniveyor_gazebo_world)/launch/singleRobot.launch">
      <arg name="x" value="1.0"/>
      <arg name="y" value="0.0"/>
    </include>
    ```

### Operation Tutorial:

#### Demos 101:
  - Gazebo example 1: The differential-drive robot is set up for manually specifying joint commands in the Gazebo GUI.
  - Gazebo example 2: The treaded robot is set up to receive Twist command from ROS. Try (use `Tab` key to autofill the message content, or just sequentially supply six floating point values):
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

#### To operate the OmniVeyor robot:
  - The OmniVeyor is set up with a full suite of LiDAR, camera, IMU, and odometry sensors. To control its motion and receive its wheel odometry, try (again, use `Tab` key to autofill the message content, or just sequentially supply six floating point values):
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
  - To fetch LiDAR scans, refer to the `/scan` topic.
  - To fetch camera RGB and depth images, refer to the `/cam_d1/depth/image_rect_raw`, `/cam_d1/color/image_raw` topics, etc.
  - Referring to the launch file and urdf/xacro files in dependent package `omniveyor_common` for detailed explanation on available options.

#### List of useful topics:
  The robot operates with the following topics:
  - `/cmd_vel` :arrow_left: type: `geometry_msgs/Twist`, velocity command to the OmniVeyor robot.

  The robot provides the following topics:
  - `/tf_static` :arrow_right: static transformations from the robot base link (geometric center, projected to the ground) to all sensors (one LiDAR, three optional cameras with builtin Inertial Measurement Units) onboard. The robot's URDF definition is provided in the `omniveyor_common` package.
  - `/tf` :arrow_right: transformations between relatively moving frames. The OmniVeyor simulator provides the `odom` --> `base_link` transform, specifically.
  - `/odom` :arrow_right: type: `nav_msgs/Odometry`, cumulative motion calculated from integrating wheel rotations. Updates at $50$ Hz.
  - `/scan` :arrow_right: type: `sensor_msgs/Scan`, 2-D LiDAR scan provided by a simulated Hokuyo LiDAR located on the bottom of the robot. It provides distances to surroundings on the front half circle (field of view: right 90 degs --> front --> left 90 degs). Updates at $40$ Hz.
  - (Optional) `/cam_d1/*` namespace :arrow_right: topics mimicking an Intel RealSense D435i camera, mounted on the top of the robot and facing front. Topics inside this namespace, as the topic names suggest, include a color image, a depth image, and an IMU readout.
  - (Optional) `/cam_d2/*` namespace :arrow_right: topics mimicking an Intel RealSense D435i camera, mounted on the top of the robot and facing rear. Topics inside this namespace, as the topic names suggest, include a color image, a depth image, and an IMU readout.
  - (Optional) `/cam_t1/*` namespace :arrow_right: topics mimicking an Intel Tracking T265 camera, mounted on the top of the robot and facing front. The camera performs internal visual-inertial odometry. Topics inside this namespace, as the topic names suggest, include video streams from two fisheye cameras, an IMU readout, and a fused visual odometry calculation.

#### Consistency with the physical robot:
  (requires [standard setup procedure and dependency packages](https://github.com/HaoguangYang/omniveyor))
  ```sh
  roslaunch omniveyor_gazebo_world omniveyorSim.launch world:=IMI-World keyboard_teleop:=1
  ```
  If you have a joystick, you can also try:
  ```sh
  roslaunch omniveyor_gazebo_world omniveyorSim.launch world:=IMI-World joystick_teleop:=1
  ```
  -  This starts the simulation of the robot with the same sensor setup with the physical robot (compared with `roslaunch pcv_base run_bare_bones.launch`), in the world file specified by the `world` argument.

#### Multi-robot operation example:
  (requires [standard setup procedure and dependency packages](https://github.com/HaoguangYang/omniveyor))
  ```sh
  roslaunch omniveyor_gazebo_world parallelManipulator.launch
  ```
  To plug in the jacobian-based platooning algorithm, run:
  ```sh
  roslaunch omniveyor_gazebo_world parallelManipulator.launch control:=1
  ```
  - If multiple OmniVeyors are spawned, the topics are encapsulated under the robot's namespace.

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
