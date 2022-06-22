# nav2_speed_filter
## Introduction
Navigation in ROS2 with Nav2 introduced a lot of new features and possibilities. The goal of this rospackage is to explore and test the new feature of navigating in a known environment where certain portions of the map have fixed speed limits requirements that the robot should meet.
<p align="center">
  <img src="https://user-images.githubusercontent.com/65722399/175064847-87e9ac62-ef55-423f-9be3-e37f5c605d0c.gif" width="350" title="speed_filter">
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
colcon build --symlink-install --packages-select
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
