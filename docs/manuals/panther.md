---
title: Panther
id: panther
---

## Overview ##

Autonomous, mobile robot platform dedicated for outdoor environment. Depending on the use-case it can be equipped with a robot arm, LIDAR, RGB-D camera, GPS, UWB and other additional equipment. It can be used in various areas of application such as agriculture, rescue, inspection and many more.

## Hardware guide ##

### Specification ###

**Dimensions and weight**

|          Name          	|  Value 	|
|:----------------------:	|:------:	|
|                 lenght 	| 805 mm 	|
|                  width 	| 840 mm 	|
|        platform height 	| 290 mm 	|
|         overall height 	| 365 mm 	|
|              wheelbase 	| 440 mm 	|
|        track of wheels 	| 695 mm 	|
|  max. ground clearance 	| 155mm  	|
|  min. ground clearance 	| 139 mm 	|
| up-side-down clearance 	| 75 mm  	|
|                 weight 	| 50 kg  	|

**Traction**

|              Name             	|  Value  	|
|:-----------------------------:	|:-------:	|
|                     max speed 	| 2 m/s   	|
|     maximum carrying capacity     | 80 kg     |
|          nominal shaft torque 	| 17.4 Nm 	|
|          maximum shaft torque 	| 30 Nm   	|
|  nominal total traction force 	| 365 N   	|
|  maximum total traction force 	| 630 N   	|
|               hil climb grade 	| 96%     	|
| climb grade with 50 kgs cargo 	| 40%     	|
| climb grade with 80 kgs cargo 	| 30%     	|
|          hill grade traversal 	| 80%     	|


### Components ###

**standard modules**
* signal lights,
* inertial navigation system,
* on-board computer

**external modules**
* GPS,
* headlights,
* RGBD camera,
* manipulator

**communication**
* Ethernet,
* CAN,
* RS232,
* RS485,
* WiFi,
* I2C,
* SPI,
* USB

### Power supply ###

The Panther is equipped with a set of battery cells in Lithium-Ion technology with a rated voltage of 36V and 20Ah, which gives it 740Wh of energy to use for calculations and move around in demanding terrain for about 3.5 hours. Moving the robot in a friendly terrain allows for a significant extension of the robot's working time up to 8 hours.

To meet the user's needs, the robot is equipped with 9 high-power electrical connectors that are able to provide a total of 550W of power to the user's devices.
The supply voltages available on the user Power panel are 5V with a total current limitation up to 15A (3x female XT60), 12V limited to 20A (3x female XT60) and 19V limited to 10A (3x female XT60).

Note: One of 19V output is used by NUC. 

**Power**

|          Name          	|          Value            |
|:----------------------:	|:------------------------: |
|       Battery capacity 	| 740Wh  	                |
|                Runtime 	| 3.5 h  	                |
|     Total output power 	| 1 kW  	                |
|     Maximum peak power 	| 1.8 kW 	                |
|     Power for user use 	| 5V@15A, 12V@20A, 19V@10A  |

### Charging Panther ###

In the set with the robot, we provide a dedicated 42V @ 5A charger, which the robot will charge to 80% in 4 hours, and to 100% in 7 hours. The mains-operated charger is connected directly to the robot's charging connector on its housing.

Note: If the robot is turned on and has a connected charger, the charging process may never be completed (the green LED indicating the end of charging will not light up on the charger) despite the high level of battery charge.

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
