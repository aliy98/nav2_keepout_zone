# nav2_keepout_zone
## Introduction
Navigation in ROS2 with [Nav2](https://navigation.ros.org/) introduced a lot of new features and possibilities. The goal of this rospackage is to explore and test the new feature of navigating in a known environment where certain portions of the map have fixed speed limits requirements that the robot should meet.

## Software Architucture
UML component diagram for Nav2 software architecture can be represented as follows:

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/178117445-aef32d12-337e-4854-b90c-4c34d2455a71.png" width="750" title="nav2_architecture">
</p>

Key: (a) Action - (svc) service - (/) topic

### PC (ROS2)
Software architecture components of ROS2 framework can be divided into Nav2 Nodes and Plugins:

Nav2 Nodes:

- Map Server
- AMCL
- Planner Server 
- Controller Server
- Recovery Server
- BT Navigator

Plugins:

- Global Costmap
- Local Costmap
- Spin
- Costmap Filters


#### Map Server
Load, serve, and store maps: The Map Server implements the server for handling the map load requests for the stack and host a map topic. It also implements a map saver server which will run in the background and save maps based on service requests. There exists a map saver CLI similar to ROS 1 as well for a single map save.

#### AMCL
Localize the robot on the map: AMCL implements the server for taking a static map and localizing the robot within it using an Adaptive Monte-Carlo Localizer.

#### Planner Server
Plan a path from A to B around obstacles: The Planner Server implements the server for handling the planner requests for the stack and host a map of plugin implementations. It will take in a goal and a planner plugin name to use and call the appropriate plugin to compute a path to the goal. GridBased planner gets loaded as default plugin if it is not overridden.

#### Controller Server
Control the robot as it follows the path: The Controller Server implements the server for handling the controller requests for the stack and host a map of plugin implementations. It will take in path and plugin names for controller, progress checker and goal checker to use and call the appropriate plugins.

#### Recovery Server
Compute recovery behaviors in case of failure.


#### BT Navigator
Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. The diagram below will give you a good first-look at the structure of Nav2.


Build complicated robot behaviors using behavior trees: The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose task interface. It is a Behavior Tree-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors, including recovery.

<p align="center">
<img src="https://user-images.githubusercontent.com/65722399/178110857-2a8112f4-cbf4-4a22-9c86-79beae3e684f.png" width="1000" title="overall_bt_w_breakdown">
</p>

The `Navigation` subtree mainly involves actual navigation behavior:

- calculating a path
- following a path
- contextual recovery behaviors for each of the above primary navigation behaviors

The `Recovery` subtree includes behaviors for system level failures or items that were not easily dealt with internally.

The overall BT will (hopefully) spend most of its time in the `Navigation` subtree. If either of the two main behaviors in the `Navigation` subtree fail (path calculation or path following), contextual recoveries will be attempted.
If the contextual recoveries were still not enough, the `Navigation` subtree will return `FAILURE`. The system will move on to the `Recovery` subtree to attempt to clear any system level navigation failures.
This happens until the `number_of_retries` for the parent `RecoveryNode` is exceeded (which by default is 6).

#### Nav2 Costmap 2D (Global and Local Costmap)
Convert sensor data into a costmap representation of the world: The Costmap 2D package implements a 2D grid-based costmap for environmental representations and a number of sensor processing plugins. It is used in the planner and controller servers for creating the space to check for collisions or higher cost areas to negotiate around.

#### Spin
Invokes the Spin ROS 2 action server, which is implemented by the nav2_behaviors module. It performs an in-place rotation by a given angle. This action is used in nav2 Behavior Trees as a recovery behavior.

#### Costmap Filters
It is used in order to have a specific action occur based on the location in the annotated map. This annotated map is called “filter mask”. Just like a mask overlaid on a surface, it can or cannot be same size, pose and scale as a main map. The main goal of filter mask is to provide an ability of marking areas on maps with some additional features or behavioral changes. In this case some areas of map have speed restrictions.


### Simulation Environment (Gazebo)
This project is simulated in Gazebo environment. The components of Turtlebot3 can be represnted as follows:

- Motors Drive
- Laser Scaner
- Robot State Publisher
- Wheel Odometry

#### Motors Drive
Drives the robot according to the control signal which is subscribed from `/cmd_vel`

#### Laser Scanner
Laser Scanner node publisehs the ranges data of the robot laser sensor into `/scan`

#### Robot State Publisher
This package subscribes to joint states of the robot from the joint state publisher and publishes the 3D pose of each link using the kinematic representation from the URDF model. It publishes the state of a robot to `/tf`. Once the state gets published, it is available to all components in the system that also use `/tf`.

#### Wheel Odometry
This ROS node takes the Robot Pose message and publishes it to `/odom` topic.

## Installation (Ubuntu)
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
In order to get and build the package use the following commands:
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

## Usage and Results
To simply run Filter Info Publisher Server and Map Server tuned on Turtlebot3 standard simulation written at Getting Started, launch costmap_filter_info.launch.py as follows:
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
This will bring up simulation environent in Gazebo and also Rviz. Initially, the robot has no idea where it is located on the map, therefore we have to set the pose by using the 2D Pose Estimate button in rviz. Finally, in order to set a goal point use Nav2 goal option. 

<p align="center">
  <img src="https://user-images.githubusercontent.com/65722399/178327880-8ded7e6f-fe41-4053-ac80-51c196ef59f2.gif" width="350" title="speed_filter">
</p>

The above GIF represents the result of simulation in Rviz, and shows that the robot speed decreases in the restricted areas.
