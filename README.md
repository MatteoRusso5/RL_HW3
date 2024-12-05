# RL_HW3

## :hammer: Build

Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
$ git clone https://github.com/MatteoRusso5/RL_HW3.git
```
```
$ rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package

```
$ colcon build
```
Source the setup files

```
$ source install/setup.bash
```
## :white_check_mark: Usage 🤖
### 1. Run Rviz and Gazebo with the launch file by using the velocity interface

Per runnare il mondo con l'aruco tag e interface velocity
⚙️
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

  
Per runnare il mondo con la sfera e interface velocity
 
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_aruco:=false
```
Per runnare il nodo single_simple per il positioning
```
$ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=world -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```

Per runnare il nodo single_simple per il look-at-point
```
$ ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```

## To use the Effort Controller ⚙️
### 1. Launch Gazebo with the effort controller
 ```  
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
```
### 2. Send effort commands to the robot

```
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
```

### 3. To view torques sent to the robot run 
```
$ ros2 run rqt_plot rqt_plot
```
Per runnare il nodo ros2_kdl_vision_control per task positioning
```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control
```

Per runnare il nodo ros2_kdl_vision_control per task look-at-point

```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:="look-at-point"
```

Per runnare il nodo ros2_kdl_vision_control per il punto b

```
$ ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p task:="look-at-point" -p cmd_interface:=effort
```

