---
title: Panther
id: panther
---

## Overview ##

<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_right-front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_right-front_perspective_view.jpg" alt="Panther basic right-front perspective view" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_front_perspective_view.jpg" alt="Panther basic front perspective view" class="hover-shadow"/>
    </a>
</div> 
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_left-front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_left-front_perspective_view.jpg" alt="Panther basic left-front perspective view" class="hover-shadow"/>
    </a>
</div> 
</div>

Autonomous, mobile robot platform dedicated for outdoor environment. Depending on the use-case it can be equipped with a robot arm, LIDAR, RGB-D camera, GPS, UWB and other additional equipment. It can be used in various areas of application such as agriculture, rescue, inspection and many more.


<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_left-front_high_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_left-front_high_perspective_view.jpg" alt="Panther basic left-front high perspective view" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_left_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_left_perspective_view.jpg" alt="Panther basic left perspective view" class="hover-shadow"/>
    </a>
</div> 
<div class="img-container w3">
    <a href="/docs/assets/img/Panther/Panther_basic_right-front_high_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/Panther/Panther_basic_right-front_high_perspective_view.jpg" alt="Panther basic right-front high perspective view" class="hover-shadow"/>
    </a>
</div> 
</div>


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

### Block diagram ###

Graphic representation of Panther components and connections between them. A full, more detailed version of the block diagram can be downloaded [here](https://files.husarion.com/panther/schematic_block_diagram.pdf).

![Block diagram](/docs/assets/img/Panther/simplified_block_diagram.png "Block diagram")

### Power supply ###

The Panther is equipped with a set of battery cells in Lithium-Ion technology with a rated voltage of 36V and 20Ah, which gives it 740Wh of energy to use for calculations and move around in demanding terrain for about 3.5 hours. Moving the robot in a friendly terrain allows for a significant extension of the robot's working time up to 8 hours (standby time up to 40 hours).

To meet the user's needs, the robot is equipped with 9 high-power electrical connectors that are able to provide a total of 505W of power to the user's devices.
The supply voltages available on the user Power panel are 5V with a total current limitation up to 15A (3x female XT60), 12V limited to 20A (3x female XT60) and 19V limited to 10A (3x female XT60).

Note:

* One of 19V output is used by NUC.
* One of 12V output is used by router


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
| `/panther_lights` | `panther_lights/LightsMessage` | subscriber | `/panther_lights` | Control the front and rear lights |

Lights pattern could be set by publishing appropriate message to `/panther_lights` topic, to change animation to `BLINKER_LEFT`:
```
rostopic pub /panther_lights panther_lights/LightsMessage "animation: 1
custom_color: ''" -1
```

To change animation to `BLINKER_LEFT` with custom colors (front is green, rear is red):
```
rostopic pub /panther_lights panther_lights/LightsMessage "animation: 1
custom_color: '0x00FF00 0xFF0000'" -1
```
More details regarding lights control could be found in [`panther_lights` documentation](https://github.com/byq77/panther-lights#ros-interface).

#### External documentation ####

 - Slamtec RpLidar scanner API is documented in [driver repository](https://github.com/Slamtec/rplidar_ros)

### Joystick control ###

The Raspberry Pi SBC has a preinstalled webui with simple joystick. The joystick allows user to issue simple motion commands for the robot.
To use the joystick, open `RASPBERRY_PI_IP_ADDRESS:8000`.

### System reinstallation ###

 In some cases you will need to restore Panther's system to its default settings:
 - in case of accidential damage of the system,
 - to update the OS (it can be udpated remotely, but flashing the microSD card can be easier sometimes),
 - to clear all user changes and restore factory settings.

 The reinstallation procedure is following:

1. Extract SD card from Raspberry Pi SBC by pushing card carefully until it is released back by card holder, thel pull it out. In order to find SD card slot, you will need to disassemble part of the top cover.
2. Download image for Raspberry Pi from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/production_images/ros-noetic-rpi-2020-08-18.img.xz).
3. Extract downloaded image (For this process we recommend using [unxz](https://linux.die.net/man/1/unxz) tool).
4. Flash the extracted image onto SD card (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
 - If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 
 - Download [Etcher](https://www.balena.io/etcher/) and install it.
 - Connect an SD card reader with the SD card inside.
 - Open Etcher and select from your hard drive .img file that you extracted.
 - Select the SD card you wish to write your image to.
 - Review your selections and click 'Flash!' to begin writing data to the SD card.
5. Insert SD card back to Raspberry Pi

#### Launching navigation example

The user space PC comes with preinstalled Ubuntu 20.04 and ROS, the same as the Raspberry Pi SBC. It also has a [route_admin_panel](https://github.com/husarion/route_admin_panel/) as an example application.

To start the RAP:

```
roslaunch panther_driver rap.launch 
```

Then open in browser: 

```
PANTHER_IP_ADDRESS:8000
```

You should see interface like below:

![RouteAdminPanelScreenshot](/docs/assets/img/software/route-admin-panel.png)

## Network ##

### Overview ###

Panther is equipped with a router running open-source firmware OpenWRT, which provide following interfaces:

**Ehernet**

* 1 x WAN 10/100/1000Mbps (by default configured as a LAN port)
* 3 x LAN 10/100/1000Mbps 
* 2 ports are available for user equipment

**Wireless**

* Two radios (2.4GHZ and 5Ghz)
* Support for 802.11ac (WiFi 5) with link rate up to 867Mbps, fast roaming with 802.11r
* Works as access point (AP) and/or as a client/station (STA)
* External antennas with 2dBi gain

**Cellular**

* Dual-Sim with fail-over
* LTE (4G) Cat 6
* External antenna with 3dBi gain

**GNSS**

* Support for GPS, GLONASS, Galileo and BeiDou
* Integrated into ROS

Each form of connectivity can be part of automatic WAN fail-over in order to provide continuous connection to external services.
More information is available on manufacturer [site](https://wiki.teltonika-networks.com/view/RUTX11_Manual).





## Docs and links ##
All helpful documents and links in one place:

* [Panther schematic block diagram](https://files.husarion.com/panther/schematic_block_diagram.pdf "Panther schematic block diagram") - basic robot components and connections between them,
* [Overall dimensions](https://files.husarion.com/panther/external_dimentions.pdf "Overall dimensions") - three basic projections of the platform,
* [Teltonika RUTX11 manual](https://wiki.teltonika-networks.com/view/RUTX11_Manual)
* [Teltonika RUTX11 datasheet](https://teltonika-networks.com/downloads/en/rutx11/RUTX11-Datasheet.pdf)


