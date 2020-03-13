---
title: Map navigation
sidebar_label: 9. Map navigation
id: 9-map-navigation
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

## Introduction

Map navigation is technique for navigation based on map and localization at this map.
For map creation as you have seen in previous tutorials can be used `gmapping_node`. 

## Map saving

SLAM based on for instance gmapping works this way that localization and map is available till node is running, when you close your node you will lose all your explored map.

To deal with this we can save map, this is done by `map_server` , map server is package providing all needed nodes for managing map.

### Steps for map saving

- Explore map 
    
For map exploration run `tutorial_8` for autonomous exploration or `tutorial 6/7` for doing it manually.

- Save map 

Once map is explored open another terminal and go to `tutorial_pkg/maps`, create map directory if you don't have this one.

```
roscd tutorial_pkg
mkdir maps
cd maps
rosrun map_server map_saver -f maze_map
```
This command saves map under current working directory with name specified after `-f` param.

After executing this command you should have two files:

```
maze_map.pgm  
maze_map.yaml
```
File `maze_map.yaml` contains all information about this map and it should look as follows:

```
image: maze_map.pgm
resolution: 0.010000
origin: [-5.000000, -5.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Parameters

- `image` - Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file.

- `resolution` - Resolution of the map, meters / pixel.

- `origin` - The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation).

- `negate` - Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected). 

- `occupied_thresh` - Pixels with occupancy probability greater than this threshold are considered completely occupied.

- `free_thresh` - Pixels with occupancy probability less than this threshold are considered completely free.


## Map loading

To load map we use `map_server_node`, use this in following way:

```
rosrun map_server map_server maze_map.yaml
```

Below we will create launch file for map_server.
```
<launch>
<arg name="map_file" default="$(find tutorial_pkg)/maps/maze_map.yaml"/>       <!-- path of map file -->
<node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" respawn="true" />
</launch>
```

Save this file in `launch` directory and name it `map_launch.launch`

## AMCL

AMCL is a probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as [described](https://papers.nips.cc/paper/1998-kld-sampling-adaptive-particle-filters.pdf) by Dieter Fox ), which uses a particle filter to track the pose of a robot against a known map. 

What it does is basically locating robot on map based on how map looks and how current laser scan match to map. Put simply, `AMCL + map_server` replaces `gmapping`. 


### AMCL params

There are a lot of params provided by AMCL node, we will focus on those which are most important, for all please refer to [amcl](http://wiki.ros.org/amcl)

- `scan` - Laser scan topic. 

- `global_frame_id` - Frame name of global frame, usually it's map

- `odom_frame_id` - Frame name of odom.

- `base_frame_id` - Frame name of base. 

- `odom_model_type` - Method of how your navigates options are diff/omni ,but use fixed algorithms with _-corrected_ suffix. Example: "diff-corrected".

- `update_min_d` - Translational movement required before performing a filter update. 

- `update_min_a` - Rotational movement required before performing a filter update.

- `min_particles` - Minimum allowed number of particles. How many of laser scan points is used to estimate position, if you have changing environment consider enlarging this param.

- `tf_broadcast` - Decide if publish the transform between the global frame and the odometry frame. 

- `initial_pose_x` - Initial pose mean (x), you should set this param different than "0.0" only if place where robot starts navigation is different than start point for map. If it's small distance you should leave this as "0.0".

- `initial_pose_y` - Initial pose mean (x). Use this the same way as `initial_pose_x`.

- `initial_pose_a` - Initial pose mean (yaw). Use this the same way as `initial_pose_x`.

### Initial Pose

When using `initial_pose_*` make sure where map origin is (starting point of map building) and then provide coordinates of robot relative to this point. Distance in metres and angle in the radians.

This might be little confusing especially if you are not sure where map was started to be build. For this case there is better way using `rviz`.

When you launch and the pose calculated by AMCL doesn't match real pose use `2D Estimate Pose` from `rviz`. Select point the same way as navigation goal.

![image](/docs/assets/img/ros/man-9-estimate-pose.png)

## Launch navigation

### AMCL Launch

Create launch file for AMCL based on params we have discussed, and save it under launch directory and name `amcl_only.launch` :

```
<launch>
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <remap from="scan" to="scan"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="odom_model_type" value="diff-corrected"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="update_min_d" value="0.1"/>
    <param name="update_min_a" value="0.2"/>
    <param name="min_particles" value="500"/>
    <param name="global_frame_id" value="map"/>
    <param name="tf_broadcast" value="true" />
    <param name="initial_pose_x" value="0.0"/>
    <param name="initial_pose_y" value="0.0"/>
    <param name="initial_pose_a" value="0.0"/>
  </node>
   
</launch>
```

### Map Launch

Create following file in `launch` directory and name it `map_server.launch` :

```
<launch>
    <arg name="map_file" default="$(find tutorial_pkg)/maps/maze_map.yaml"/>       <!-- path of map file -->
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" respawn="true" />
</launch>
```

### Final launch

Okay, let's create one launch file combining all what we need to provide autonomous navigation based on map and amcl.

Save following file as `tutorial_9.launch` :

```
<launch>

    <arg name="use_rosbot" default="true"/>
    <arg name="use_gazebo" default="false"/>

    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/maze_world.launch"/>
    <include if="$(arg use_gazebo)" file="$(find rosbot_description)/launch/rosbot_gazebo.launch"/>

    <param if="$(arg use_gazebo)" name="use_sim_time" value="true"/>

    <node if="$(arg use_rosbot)" pkg="rplidar_ros" type="rplidarNode" name="rplidar">
        <param name="angle_compensate" type="bool" value="true"/>
        <param name="serial_baudrate" type="int" value="115200"/><!--model A2 (ROSbot 2.0) -->
        <!--<param name="serial_baudrate" type="int" value="256000"/>--><!-- model A3 (ROSbot 2.0 PRO) -->
    </node>

    <!-- ROSbot 2.0 -->
    <include if="$(arg use_rosbot)" file="$(find rosbot_ekf)/launch/all.launch"/>

    <!-- ROSbot 2.0 PRO -->
    <!-- <include file="$(find rosbot_ekf)/launch/all.launch" >
      <arg name="rosbot_pro" value="true" />
    </include> -->

    <node if="$(arg use_rosbot)" pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0 3.14 0 0 base_link laser 100" />

    <node pkg="move_base" type="move_base" name="move_base" output="log">
		<param name="controller_frequency" value="10.0" />
		<rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
		<rosparam file="$(find tutorial_pkg)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
		<rosparam file="$(find tutorial_pkg)/config/local_costmap_params.yaml" command="load" />
		<rosparam file="$(find tutorial_pkg)/config/global_costmap_params.yaml" command="load" />
		<rosparam file="$(find tutorial_pkg)/config/trajectory_planner.yaml" command="load" />
	</node>

    <include file="$(find tutorial_pkg)/launch/map_server.launch"/>

    <include file="$(find tutorial_pkg)/launch/amcl_only.launch"/>

</launch>
```
Launch this with commands:


ROSbot:

```
roslaunch tutorial_pkg tutorial_9.launch 
```

Gazebo:

```
roslaunch tutorial_pkg tutorial_9.launch use_rosbot:=false use_gazebo:=true
```

Then open another terminal and check out how it works with `rviz`. You should be able to set target using `2D Nav Goal` and robot should drive there. 

![image](/docs/assets/img/ros/man-9-rviz.png)

When you create new map change map path to proper at `map_server.launch` file. 


## Summary

After completing this tutorial you should be able to configure
`AMCL` and use `map_server` nodes to navigate robot at previously created map.

---

_by Adam Krawczyk, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_


