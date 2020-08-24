---
title: Panther
id: panther
---

## Overview ##

## Hardware guide ##

## Specification ##

## Components ##

### Components description ###

## Power supply ##

## Charging Panther ##

## Software ##

Panther robot is equipped with the Raspberry Pi 4 SBC with custom OS based on Ubuntu 20.04 and contains all components needed to start working with ROS immediately. The microSD card with OS for the Raspberry Pi is included with each Panther robot. The OS contains software drivers for all components and has been modified to make the file system insensitive to sudden power cuts.

### ROS API ###

Below are topics and services available in ROSbot:

| Topic | Message type | Direction | Node |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- | --- |
| `/joint_states`| `sensor_msgs/JointState` | publisher | `/panther_driver` | Wheels angular position [encoder pulses], speed [encoder pulses per second] and motor current [Amperes] oredered as [Front left, Front right, Rear left, Rear right]|
| `/battery` | `sensor_msgs/BatteryState` | publisher | `/panther_driver` | Battery voltage |
| `/pose` | `geometry_msgs/Pose` | publisher | `/panther_driver` | Position based on encoders |
| `/cmd_vel` | `geometry_msgs/Twist` | subscriber | `/panther_driver` | Velocity commands |

#### External documentation ####

 - Slamtec RpLidar scanner API is documented in [driver repository](https://github.com/Slamtec/rplidar_ros)

### ROS2 API

#### External documentation

### System reinstallation ###

#### Launching navigation example on ROS2 Dashing

## Docs and links ##
All helpful documents and links in one place:
