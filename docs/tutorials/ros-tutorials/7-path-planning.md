---
title: Path planning
sidebar_label: 7. Path planning
id: 7-path-planning
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

We have prepared ready to go virtual environment with end effect of following this tutorial. It is available on ROSDS:

<div><center>
<a href="http://www.rosject.io/l/b968f74/">
<img alt="run-on-ROSDS" src="/img/ros/Run-on-ROSDS-button.png" width="250px"/></a>
</center></div>

## Introduction

Task of path planning for mobile robot is to determine sequence of
manoeuvrers to be taken by robot in order to move from starting point to
destination avoiding collision with obstacles.

Sample algorithms for path planning are:

- Dijkstra’s algorithm

- A\*

- D\*

- Artificial potential field method

- Visibility graph method

Path planning algorithms may be based on graph or occupancy grid.

### Graph methods

Method that is using graphs, defines places where robot can be and
possibilities to traverse between these places. In this representation
graph vertices define places e.g. rooms in building while edges define
paths between them e.g. doors connecting rooms. Moreover each edge can
have assigned weights representing difficulty of traversing path e.g.
door width or energy required to open it. Finding the trajectory is
based on finding the shortest path between two vertices while one of
them is robot current position and second is destination.

### Occupancy grid methods

Method that is using occupancy grid divides area into cells (e.g. map
pixels) and assign them as occupied or free. One of cells is marked as
robot position and another as a destination. Finding the trajectory is
based on finding shortest line that do not cross any of occupied cells.

## Occupancy grid path planning in ROS

In ROS it is possible to plan a path based on occupancy grid, e.g. the
one obtained from `slam_gmapping`. Path planner is `move_base` node from
`move_base` package.

### Requirements regarding robot

Before continuing with `move_base` node certain requirements must be
met, robot should:

- subscribe `cmd_vel` topic with message type `geometry_msgs/Twist` in
  which robot desired velocities are included.

- Publish to `/tf` topic transformations between robot relative to
  starting point and laser scanner relative to robot.

- Publish map to `/map` topic with message type
  `nav_msgs/OccupancyGrid`

Above configuration is met by the robot created in previous manual.

## Configuration of `move_base` node

`Move_base` node creates cost map basing on occupancy grid. Cost map
is a grid in which every cell gets assigned value (cost) determining
distance to obstacle, where higher value means closer distance. With
this map, trajectory passing cells with lowest cost is generated.
`Move_base` node uses two cost maps, local for determining current
motion and global for trajectory with longer range.

For `move_base` node some parameters for cost map and trajectory planner
need to be defined, they are stored in `.yaml` files.

### Common parameters for cost map

Common parameters are used both by local and global cost map. We will
define following parameters:

```yaml
obstacle_range: 6.0
```
In this range obstacles will be considered during path planning.

```yaml
raytrace_range: 8.5
```

This parameter defines range in which area could be considered as free.

```yaml
footprint: [[0.12, 0.14], [0.12, -0.14], [-0.12, -0.14], [-0.12, 0.14]]
```
This parameter defines coordinates of robot outline, this will
considered during collision detecting.

```yaml
map_topic: /map
```
This parameter defines topic where occupancy grid is published.

```yaml
subscribe_to_updates: true
```
This parameter defines if `move_base` should periodically check if map
was updated.

```yaml
observation_sources: laser_scan_sensor
```
This parameter defines type of sensor used to provide data.

```yaml
laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true, min_obstacle_height: 0.0, max_obstacle_height: 5.0, obstacle_range: 6.0, raytrace_range: 8.5}
```
This parameter define properties of used sensor, these are:

- `sensor_frame` - coordinate frame tied to sensor

- `data_type` - type of message published by sensor

- `topic` - name of topic where sensor data is published

- `marking` - true if sensor can be used to mark area as occupied

- `clearing` - true if sensor can be used to mark area as clear

- `min_obstacle_height` - the minimum height in meters of a sensor reading considered valid

- `max_obstacle_height` - the maximum height in meters of a sensor reading considered valid

- `obstacle_range` - the maximum range in meters at which to insert obstacles into the costmap using sensor data

- `raytrace_range` - the maximum range in meters at which to raytrace out obstacles from the map using sensor data

```yaml
global_frame: map
```

This parameter defines coordinate frame tied to occupancy grid map.

```yaml
robot_base_frame: base_link
```

This parameter defines coordinate frame tied to robot.

```yaml
always_send_full_costmap: true
```

This parameter define if costmap should be always published with complete data.

```yaml
plugins: 
```
This parameter groups following parameters to be considered as plugins.

```yaml
- {name: inflation_layer,        type: "costmap_2d::InflationLayer"}
- {name: obstacle_layer,         type: "costmap_2d::ObstacleLayer"}
```
Here we specify which spacific layers we want to use.

```yaml
static_layer:
    map_topic: /map
    subscribe_to_updates: true
obstacle_layer:
    observation_sources: laser_scan_sensor
    laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true, min_obstacle_height: 0.0, max_obstacle_height: 5.0, obstacle_range: 6.0, raytrace_range: 8.5}
```

Here we specify how particular layer will look.

This parameter define if costmap should be always published with complete data.

Your final file should look like below:

```yaml
obstacle_range: 6.0
raytrace_range: 8.5
footprint: [[0.12, 0.14], [0.12, -0.14], [-0.12, -0.14], [-0.12, 0.14]]
map_topic: /map
subscribe_to_updates: true
global_frame: map
robot_base_frame: base_link
always_send_full_costmap: true
static_layer:
    map_topic: /map
    subscribe_to_updates: true
plugins:
    - {name: inflation_layer,        type: "costmap_2d::InflationLayer"}
    - {name: obstacle_layer,         type: "costmap_2d::ObstacleLayer"}
obstacle_layer:
    observation_sources: laser_scan_sensor
    laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true, min_obstacle_height: 0.0, max_obstacle_height: 5.0, obstacle_range: 6.0, raytrace_range: 8.5}
```

Save it as `costmap_common_params.yaml` in `tutorial_pkg/config` directory.

### Parameters for local cost map

These parameters are used only by local cost map. We will define
following parameters:

```yaml
local_costmap:
```
This parameter groups following parameters to be considered only by
local planner.

```yaml
update_frequency: 5
```
This parameter defines how often cost should be recalculated.

```yaml
publish_frequency: 5
```

This parameter defines how often cost map should be published to topic.

```yaml
transform_tolerance: 0.25
```
This parameter define latency in published transforms (in seconds), if
transforms are older than this, planner will stop.

```yaml
static_map: false
```

This parameter defines if map can change in time, true if map will not
change.

```yaml
rolling_window: true
```

This parameter defines if map should follow position of robot.

```yaml
width: 3
height: 3
```

These parameters define size of map (in meters).

```yaml
origin_x: -1.5
origin_y: -1.5
```
These parameters define position of left bottom map corner (in meters).
If these values are half of map size, and `rolling_window` is set to
`true`, then robot will always be in cost map centre.

```yaml
resolution: 0.1
```

This parameter define size of single map cell (in meters).

```yaml
inflation_radius: 1.0
```
This parameter defines distance to obstacle where cost should be
considered, any further from obstacle than this value will be treated as
no cost.

```yaml
plugins: 
```
This parameter groups following parameters to be considered as plugins.

```yaml
- {name: obstacle_layer,        type: "costmap_2d::VoxelLayer"}
```
Here we specify which spacific layer we want to use.

Your final file should look like below:

```yaml
local_costmap:
  update_frequency: 5
  publish_frequency: 5
  transform_tolerance: 0.25
  static_map: false
  rolling_window: true
  width: 3
  height: 3
  origin_x: -1.5
  origin_y: -1.5
  resolution: 0.1
  inflation_radius: 0.6
  plugins:
      - {name: obstacle_layer,        type: "costmap_2d::VoxelLayer"}
```

Save it as `local_costmap_params.yaml` in `tutorial_pkg/config` directory.

### Parameters for global cost map

These parameters are used only by global cost map. Parameter meaning is
the same as for local cost map, but values may be different.

Your file for global cost map should look like below:

```yaml
global_costmap:
  update_frequency: 2.5
  publish_frequency: 2.5
  transform_tolerance: 0.5
  width: 15
  height: 15
  origin_x: -7.5
  origin_y: -7.5
  rolling_window: true
  inflation_radius: 2.5
  resolution: 0.1
  plugins:
   - {name: static_layer,        type: "costmap_2d::StaticLayer"}
```

Save it as `global_costmap_params.yaml` in `tutorial_pkg/config` directory.

### Parameters for trajectory planner

These parameters are used by trajectory planner. We will define
following parameters:

```yaml
TrajectoryPlannerROS:
```
This parameter groups following parameters to be considered only by
trajectory planner.

```yaml
max_vel_x: 0.2
```

This parameter defines maximum linear velocity that will be set by
trajectory planner.

```yaml
min_vel_x: 0.1
```

This parameter defines minimum linear velocity that will be set by
trajectory planner. This should be adjusted to overcome rolling
resistance and other forces that may suppress robot from moving.

```yaml
max_vel_theta: 0.35
min_vel_theta: -0.35
```

This parameter defines maximum angular velocity that will be set by
trajectory planner.

```yaml
min_in_place_vel_theta: 0.25
```

This parameter defines minimum angular velocity that will be set by
trajectory planner. This should be adjusted to overcome rolling
resistance and other forces that may suppress robot from moving.

```yaml
acc_lim_theta: 0.25
acc_lim_x: 2.5
acc_lim_Y: 2.5
```

These parameters define maximum values of accelerations used by
trajectory planner.

```yaml
holonomic_robot: false
```

This parameter defines if robot is holonomic.

```yaml
meter_scoring: true
```

This parameter defines if cost function arguments are expressed in map
cells or meters (if true, meters are considered).

```yaml
xy_goal_tolerance: 0.15
yaw_goal_tolerance: 0.25
```

These parameters define how far from destination it can be considered as
reached. Linear tolerance is in meters, angular tolerance is in radians.

Your final file should look like below:

```yaml
TrajectoryPlannerROS:
  max_vel_x: 0.2
  min_vel_x: 0.1
  max_vel_theta: 0.35
  min_vel_theta: -0.35
  min_in_place_vel_theta: 0.25

  acc_lim_theta: 0.25
  acc_lim_x: 2.5
  acc_lim_Y: 2.5

  holonomic_robot: false

  meter_scoring: true

  xy_goal_tolerance: 0.15
  yaw_goal_tolerance: 0.25
```

Save it as `trajectory_planner.yaml` in `tutorial_pkg/config` directory.

Remember that you may need to adjust cost map and trajectory planner
parameters to your robot and area that you want to explore.

### Launching path planning node

To test above configuration you will need to run `move_base` node with
nodes from SLAM configuration, you will not need only
`teleop_twist_keyboard` as commands now will be issued by trajectory
planner.

To sum up, you will need to run following nodes:

- `CORE2` bridge node -
  `roslaunch rosbot_ekf all.launch` - publishes tf, connect to CORE2 and run extended Kalman filter for odometry.

- `rplidarNode` - driver for rpLidar laser scanner

Or instead of these, `Gazebo`:

- `roslaunch rosbot_gazebo maze_world.launch`

And:

- `static_transform_publisher` - `tf` publisher for transformation of
  laser scanner relative to robot

- `slam_gmapping` - map building node

- `move_base` - trajectory planner

- `rviz` - visualization tool

For the `move_base` node you will need to specify paths for `.yaml`
configuration files.

Set frequency for trajectory generation:

```xml
<param name="controller_frequency" value="10.0"/>
```
Load common parameters for global cost map:

```xml
<rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
```

Load common parameters for local cost map:

```xml
<rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
```

Load only local cost map parameters:

```xml
<rosparam file="$(find tutorial_pkg)/config/local_costmap_params.yaml" command="load" />
```
Load only global cost map parameters:

```xml
<rosparam file="$(find tutorial_pkg)/config/global_costmap_params.yaml" command="load" />
```

Load trajectory planner parameters:

```xml
<rosparam file="$(find tutorial_pkg)/config/trajectory_planner.yaml" command="load" />
```
You can use below `launch` file:

```xml
<launch>

  <arg name="rosbot_pro" default="false" />
  <arg name="use_gazebo" default="false" />

  <!-- Gazebo -->
  <group if="$(arg use_gazebo)">
    <include file="$(find rosbot_gazebo)/launch/maze_world.launch" />
    <include file="$(find rosbot_description)/launch/rosbot_gazebo.launch"/>
    <param name="use_sim_time" value="true" />
  </group>

  <!-- ROSbot 2.0 -->
  <group unless="$(arg use_gazebo)">
    <include file="$(find rosbot_ekf)/launch/all.launch">
      <arg name="rosbot_pro" value="$(arg rosbot_pro)" />
    </include>

    <include if="$(arg rosbot_pro)" file="$(find rplidar_ros)/launch/rplidar_a3.launch" />
    <include unless="$(arg rosbot_pro)" file="$(find rplidar_ros)/launch/rplidar.launch" />
  </group>

  <node unless="$(arg use_gazebo)" pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0 3.14 0 0 base_link laser 100" />

  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard" output="screen"/>

  <node pkg="rviz" type="rviz" name="rviz"/>

  <node pkg="gmapping" type="slam_gmapping" name="gmapping">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom" />
    <param name="delta" value="0.1" />
  </node>

  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <param name="controller_frequency" value="10.0"/>
    <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find tutorial_pkg)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find tutorial_pkg)/config/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find tutorial_pkg)/config/trajectory_planner.yaml" command="load" />
  </node>

</launch>
```

### Setting goal for trajectory planner

After you launched trajectory planner with all accompanying nodes, your
robot still will not be moving anywhere. You have to specify target
position first.

We will begin with visualization of robot surrounding, cost maps and
planned trajectory. Go to `rviz`, add `Tf`, `/scan` and `/map`, again
open object adding window, go to tab **`By topic`** and from the list
select

- `move_base/TrajectoryPlannerROS/local_plan/Path`

- `move_base/TrajectoryPlannerROS/global_plan/Path`

- `move_base/global_costmap/footprint/Polygon`

Then for global plan path change its colour to red (values 255; 0; 0):

![image](/img/ros/man_7_1.png)

Now you can aadd one more element, open object adding window, go to tab
**`By topic`** and from the list select `/move_base_simple/goal/Pose`,
this will visualize destination point for your robot, it will not appear
until you set destination.

If you want to observe obstacles that are considered in path planning
you may add two more objects, these will be local and global costmaps,
open object adding window, go to tab **`By topic`** and from the list
select `move_base/global_costmap/costmap` and
`move_base/local_costmap/costmap`. Now change parameter `Color Scheme`
of both costmaps to `costmap`, this will allow to distinguish costmaps
from occupancy grid map.

![image](/img/ros/man_7_2.png)

Having all the elements visualized, you can set goal for robot, from
**`Toolbar`** click button **`2D nav goal`**, then click a place in
**`Visualization window`**, that will be destination for your robot.
Observe as path is generated (you should see a line from your robot
pointing to direction) and robot is moving to its destination.

![image](/img/ros/man_7_3.png)

## Summary

After completing this tutorial you should be able to configure
`move_base` node to plan trajectory for your robot, visualize cost maps
and planned trajectory in rviz and finally set destination point using
rviz.

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
