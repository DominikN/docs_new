---
title: Unknown environment exploration
sidebar_label: 8. Unknown environment exploration
id: 8-unknown-environment-exploration
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

## Introduction

Environment exploration task is to determine robot desired positions in
such a sequence which gives as much information regarding environment as
possible. One of the approaches for this task is to frontiers of
occupancy grid. As a frontier we define line between free space ant
territory marked as unknown. Moving towards frontiers, unknown area can
be explored and marked as free or occupied and frontiers are moved into
an unknown territory. Process is repeated until all frontiers are
investigated, this means free area must be surrounded by occupied cells.


## Environment exploration in ROS

In ROS it is possible to explore environment with use of occupancy grid
frontiers. One of the nodes, that perform this task is `explore`
node from `explore_lite` package. This node uses occupancy grid
e.g. created by `slam_gmapping` and publishes goal position to
`/move_base/goal` topic subscribed by path planner e.g. `move_base`
node.

### Requirements regarding robot

Before continuing with `explore_lite` node certain requirements must
be met, robot should:

- subscribe `/move_base/goal` topic with message type
  `geometry_msgs/PoseStamped` in which robot desired positions are
  included.

- Publish map to `/map` topic with message type
  `nav_msgs/OccupancyGrid`

Above configuration is met by the robot created in previous manual.

## Configuration of `explore` node

Before starting experimenting with `explore_lite` you need to have working `move_base` for navigation. You should be able to navigate with `move_base` manually through `rviz`.

You should be also able to to navigate with move_base though unknown space in the map. If you set the goal to unknown place in the map, planning and navigating should work. With most planners this should work by default. Navigation through unknown space is required for `explore_lite`.

If you want to use costmap provided by `move_base` you need to enable unknown space tracking by setting `track_unknown_space: true`.

All required configuration should be set properly doing previous tutorials, so you can start experimenting with `explore_lite`. Provided `explore.launch` should work out-of-the box in most cases, but as always you might need to adjust topic names and frame names according to your setup.


Based on occupancy grid, `explore` node determines frontiers
between free and unknown area and using them determines robot
destinations.


### Parameters 
- `robot_base_frame` - The name of the base frame of the robot. This is used for determining robot position on map.

- `costmap_topic` - Specifies topic of source nav_msgs/OccupancyGrid.

- `costmap_updates_topic` - Specifies topic of source map_msgs/OccupancyGridUpdate. Not necessary if source of map is always publishing full updates, i.e. does not provide this topic. 

- `visualize` - Specifies whether or not publish visualized frontiers. 

- `planner_frequency` - Rate in Hz at which new frontiers will computed and goal reconsidered. 

- `progress_timeout` - Time in seconds. When robot do not make any progress for progress_timeout, current goal will be abandoned. 

- `potential_scale` - Used for weighting frontiers. This multiplicative parameter affects frontier potential component of the frontier weight (distance to frontier). 

- `orientation_scale` - Used for weighting frontiers. This multiplicative parameter affects frontier orientation component of the frontier weight.

- `gain_scale` - Used for weighting frontiers. This multiplicative parameter affects frontier gain component of the frontier weight (frontier size). 

- `transform_tolerance` - Transform tolerance to use when transforming robot pose. 

- `min_frontier_size` - Minimum size of the frontier to consider the frontier as the exploration goal. Value is in meter.



Save configuration as `exploration.yaml` in `tutorial_pkg/config` directory.

Your file should look like below:

```yaml
robot_base_frame: base_link
costmap_topic: map
costmap_updates_topic: map_updates
visualize: true
planner_frequency: 0.33
progress_timeout: 30.0
potential_scale: 3.0
orientation_scale: 0.0
gain_scale: 1.0
transform_tolerance: 0.3
min_frontier_size: 0.75

```

### Launching exploration task

To test above configuration you will need to run `explore_lite` node
with nodes from path planning configuration.

To remind, you will need to run following nodes:

- `CORE2` bridge node -
  `roslaunch rosbot_ekf all.launch` - publishes tf, connect to CORE2 and run extended Kalman filter for odometry.

- `rplidarNode` - driver for rpLidar laser scanner

Or instead ot these, `Gazebo`:

- `roslaunch rosbot_gazebo maze_world.launch`

And:

- `static_transform_publisher` - `tf` publisher for transformation of
  laser scanner relative to robot

- `slam_gmapping` - map building node

- `move_base` - trajectory planner

- `explore_lite` - exploration task

- `rviz` - visualization tool

For the `explore_lite` node you will need to specify path for `.yaml` configuration file:

```xml
	<node pkg="explore_lite" type="explore" respawn="false" name="explore" output="screen">
		<rosparam file="$(find tutorial_pkg)/config/exploration.yaml" command="load" />
	</node>
```


You can use below `launch` file, save this as `tutorial_8.launch` :

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

	
	<node pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0 3.14 0 0 base_link laser 100" />
	<node pkg="tf" type="static_transform_publisher" name="camera_publisher" args="-0.03 0 0.18 0 0 0 base_link camera_link 100" />

 	<node pkg="gmapping" type="slam_gmapping" name="gmapping_node" output="log">
        <param name="base_frame" value="base_link" />
        <param name="odom_frame" value="odom" />
        <param name="delta" value="0.01" />
        <param name="xmin" value="-5" />
        <param name="ymin" value="-5" />
        <param name="xmax" value="5" />
        <param name="ymax" value="5" />
        <param name="maxUrange" value="5" />
        <param name="map_update_interval" value="1" />
        <param name="linearUpdate" value="0.05" />
        <param name="angularUpdate" value="0.05" />
        <param name="temporalUpdate" value="0.1" />
        <param name="particles" value="100" />
    </node>
    
    <node pkg="move_base" type="move_base" name="move_base" output="log">
        <param name="controller_frequency" value="10.0" />
        <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find tutorial_pkg)/config/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find tutorial_pkg)/config/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find tutorial_pkg)/config/trajectory_planner.yaml" command="load" />
    </node>
	
	<node pkg="explore_lite" type="explore" respawn="true" name="explore" output="screen">
		<rosparam file="$(find tutorial_pkg)/config/exploration.yaml" command="load" />
	</node>

	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find tutorial_pkg)/rviz/tutorial_8.rviz"/>

</launch>
```

Launch this with commands:

ROSbot 2.0:

```
roslaunch tutorial_pkg tutorial_8.launch 
```

ROSbot 2.0 PRO:

```
roslaunch tutorial_pkg tutorial_8.launch rosbot_pro:=true
```

Gazebo:

```
roslaunch tutorial_pkg tutorial_8.launch use_gazebo:=true
```


## Explore

If everything was set correctly exploration will start immediately after node initialization. Exploration will finish when whole area is discovered. 

If you are using gazebo you should see a maze with rosbot:

![image](/docs/assets/img/ros/man-8-gazebo.png)

Inspect how it works using rviz:

![image](/docs/assets/img/ros/man-8-rviz.png)


## Summary

After completing this tutorial you should be able to configure
`explore_lite` node to find frontiers on occupancy grid map and set
goals for trajectory planner to explore all unknown area.

---

_by Adam Krawczyk, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_

