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
