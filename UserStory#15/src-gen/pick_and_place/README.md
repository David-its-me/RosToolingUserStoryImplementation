# pick_and_place

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- robot_state_publisher_node
- controller_manager
- initial_joint_controller
- initial_joint_controller_spawner_stopped
- ur_ros2_control_node
- dashboard_client
- urscript_interface
- controller_stopper
- moveit_simple_controller_manager


## Installation

### Using release

This package can be copied to a valid ROS 2 workspace. To be sure that all the related dependencies are intalles the command **rosdep install** can be used.
Then the workspace must be compiled using the common ROS 2 build command:

```
mkdir -p ros2_ws/src
cd ros2_ws/
cp PATHtoTHISPackage/pick_and_place src/. 
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```


### From source code
```
mkdir -p ros2_ws/src
cd ros2_ws/
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Drivergit clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Drivergit clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Drivergit clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```

## Usage


To execute the launch file, the following command can be called:

```
ros2 launch pick_and_place pick_and_place.launch.py
```

The generated launch files requires the xterm package, it can be installed by:

```
sudo apt install xterm
```



