# nav2_speed_filter
## Introduction
Navigation in ROS2 with [Nav2](https://navigation.ros.org/) introduced a lot of new features and possibilities. The goal of this rospackage is to explore and test the new feature of navigating in a known environment where certain portions of the map have fixed speed limits requirements that the robot should meet.

<p align="center">
  <img src="https://user-images.githubusercontent.com/65722399/175064847-87e9ac62-ef55-423f-9be3-e37f5c605d0c.gif" width="350" title="speed_filter">
</p>

## Nav2 Architucture
Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. The diagram below will give you a good first-look at the structure of Nav2.
It has tools to:

- load, serve, and store maps (Map Server)
- localize the robot on the map (AMCL)
- plan a path from A to B around obstacles (Nav2 Planner)
- control the robot as it follows the path (Nav2 Controller)
- Smooth path plans to be more continuous and feasible (Nav2 Smoother)
- convert sensor data into a costmap representation of the world (Nav2 Costmap 2D)
- build complicated robot behaviors using behavior trees (Nav2 Behavior Trees and BT Navigator)
- Compute recovery behaviors in case of failure (Nav2 Recoveries)
- Follow sequential waypoints (Nav2 Waypoint Follower)
- Manage the lifecycle and watchdog for the servers (Nav2 Lifecycle Manager)
- Plugins to enable your own custom algorithms and behaviors (Nav2 Core)

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/176279400-3412daac-7d11-42ac-8e68-7ff7c0942104.png" width="500" title="nav2_architecture">
</p>

## Speed Filter
There are a number of plugin interfaces for users to create their own custom applications or algorithms with. Namely, the costmap layer, planner, controller, behavior tree, and behavior plugins. This package focuses on costmap filters plugin and in particular, speed filter, which limits maximum velocity of robot in speed restriction areas.
Each costmap filter reads incoming meta-information (such as filter type or data conversion coefficients) in messages of `nav2_msgs/CostmapFilterInfo` type. These messages are being published by Costmap Filter Info Publisher Server. The server is running as a lifecycle node. According to the diagram below, `nav2_msgs/CostmapFilterInfo` messages are going in a pair with `OccupancyGrid` filter mask topic. Therefore, along with Costmap Filter Info Publisher Server there should be enabled a new instance of Map Server configured to publish filter masks.
In order to enable Speed Filter in your configuration, both servers should be enabled as lifecycle nodes in Python launch-file

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/176288102-cd30080f-086f-40b2-92fa-8f6e5058a6fb.png" width="500" title="design">
</p>

## Usage
Install the Nav2 packages using your operating system’s package manager:
```bashscript
sudo apt install ros-<ros2-distro>-navigation2
```
Install the Turtlebot 3 packages:
```bashscript
sudo apt install ros-<ros2-distro>-turtlebot3*
```
Add the following commands in ~/.bashrc:
```bashscript
source /opt/ros/<ros2-distro>/setup.bash
```
```bashscript
export TURTLEBOT3_MODEL=waffle
```
```bashscript
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
```
To simply run Filter Info Publisher Server and Map Server tuned on Turtlebot3 standard simulation written at Getting Started, build the demo and launch costmap_filter_info.launch.py as follows:
```bashscript
mkdir -p ~/colcon_ws/src
```
```bashscript
cd ~/colcon_ws/src
```
```bashscript
git clone https://github.com/aliy98/nav2_speed_filter
```
```bashscript
cd ~/colcon_ws
```
```bashscript
colcon build --symlink-install 
```
```bashscript
source ~/colcon_ws/install/setup.bash
```
```bashscript
ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py params_file:=src/nav2_speed_filter/params/speed_params.yaml mask:=src/nav2_speed_filter/maps/speed_mask.yaml
```
After Costmap Filter Info Publisher Server and Map Server were launched and Speed Filter was enabled for global/local costmap, to run Nav2 stack, open another termianl and enter the following commands:
```bashscript
source ~/colcon_ws/install/setup.bash
```
```bashscript
ros2 launch nav2_bringup tb3_simulation_launch.py
```
