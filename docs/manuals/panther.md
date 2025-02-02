---
title: Panther
id: panther
---

<div align="center">
<iframe width="784" height="441" src="https://www.youtube.com/embed/72sSM0DN9YY" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

## Overview ##

<div class="clearfix">
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_right-front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_right-front_perspective_view.jpg" alt="Panther basic right-front perspective view" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_front_perspective_view.jpg" alt="Panther basic front perspective view" class="hover-shadow"/>
    </a>
</div> 
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_left-front_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_left-front_perspective_view.jpg" alt="Panther basic left-front perspective view" class="hover-shadow"/>
    </a>
</div> 
</div>

Autonomous, mobile robot platform dedicated for outdoor environment. Compliant with IP54 or IP66 rate of protection. Depending on the use-case it can be equipped with a robot arm, LIDAR, RGB-D camera, GPS, UWB and other additional equipment. It can be used in various areas of application such as agriculture, rescue, inspection and many more.


<div class="clearfix">
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_left-front_high_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_left-front_high_perspective_view.jpg" alt="Panther basic left-front high perspective view" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_left_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_left_perspective_view.jpg" alt="Panther basic left perspective view" class="hover-shadow"/>
    </a>
</div> 
<div class="img-container w3">
    <a href="/img/Panther/Panther_basic_right-front_high_perspective_view.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/img/Panther/Panther_basic_right-front_high_perspective_view.jpg" alt="Panther basic right-front high perspective view" class="hover-shadow"/>
    </a>
</div> 
</div>

## Quick Start ##
Here is a very basic demo allowing you to use your Panther the first time without coding:

Rotate emergency push button + set power switch to right (it is 3 pose switch, set to the 3rd pose), and then wait for Wi-Fi to come up:


> **SSID:** `Panther_<serial_number>`
>
> **PASS:** `husarion`

Connect to Wi-Fi and open **WEBUI ROS JOYSTICK** [`10.15.20.2:8000`] that allows you to manually control your robot. You can also test **Route Admin Panel** that will be available under [`10.15.20.3:8000`] address (default addresses and ports for Panther). 

When the robot is ready to work you should see it's lights flashing.

[Route Admin Panel](/software/route-admin-panel/) (RAP) is a preinstalled, open-source web user interface available on the Panther. By using RAP you can test autonomous drive of your robot - you can define navigation points and send orders to Panther to visit them. Before using RAP, at first you need to create a basic map of the environment you are going to test the robot - you can do it manually by a web-joystick available under [`10.15.20.2:8000`].


> **WARNING:** RAP is not a production-ready software, **that's a basic demo you use on your own risk**. Especially some obstacles in your working environment might be invisible for sensors in your configuration. Feel free to modify this demo code. It is open source and available on Husarion's github https://github.com/husarion/route_admin_panel .
> Also be careful while using a WEBUI ROS JOYSTICK. During testing Panther, especially during a first run, be prepared to push the emergency button rapidly if needed. Panther is quite heavy robot with high power motors, so be carefull while working with it. 


## Remote Access
You can access your Panther over the Internet, from any place in the world, thanks to Husarnet VPN service (husarnet.com). Husarnet is preinstalled on the Panther. To access your robot from any place in the world, just follow these steps:

Create an account at https://app.husarnet.com/ and click **[Create network]** button. Go to the newly created Husarnet network and click **[Add element]** button. Go to a **[join code]** tab and copy your join code, it should look like this:

`fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxxxxx`

Open a Linux terminal in your laptop that is connected to the hotspot provided by Panther

```$ ssh husarion@10.15.20.3``` with password `husarion`
To to connect to 2.4GHz network edit file ```~/panther_rutx11/config.json```

``` nano panther_rutx11/config.json```

Edit section section named `wifi_client` filling in your SSID and matching password.

Save BY pressing `Ctrl+O` and exit by `Ctrl+X`. To apply new settings execute python script by command:

``` panther_rutx11/setup.py```

When Panther connect to our network message `Success` will be shown.

> **Note:** Wi-Fi must be in range of Panther. For more information head to [Network Section](#Network)

```$ ssh husarion@10.15.20.3``` with password `husarion`

To join Panther to your Husarnet network execute:

`husarnet join fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxxxxx myPanther`

After 20 - 60 seconds you should be able to see you panther available at https://app.husarnet.com/. You can ping it:

`$ ping6 myPanther`

or SSH to it:

`$ ssh husarion@myPanther`
Remember to connect also your laptop to the same Husarnet network as Panther (https://docs.husarnet.com/docs/begin-linux).


## Hardware guide ##

### Specification ###

**Dimensions and weight (WH01 option)**

|          Name          	|  Value 	|
|:----------------------:	|:------:	|
|                 length 	| 805 mm 	|
|                  width 	| 840 mm 	|
|        platform height 	| 290 mm 	|
|         overall height 	| 365 mm 	|
|              wheelbase 	| 440 mm 	|
|        track of wheels 	| 695 mm 	|
|  max. ground clearance 	| 155mm  	|
|  min. ground clearance 	| 139 mm 	|
| up-side-down clearance 	| 75 mm  	|
|                 weight 	| 55 kg  	|
|       protection index 	| IP54  	|

**Traction**

|              Name             	|  Value  	|
|:-----------------------------:	|:-------:	|
|                     max speed 	| 2 m/s   	|
|     maximum carrying capacity     | 80 kg     |
|          nominal shaft torque 	| 34.5 Nm 	|
|          maximum shaft torque 	| 60 Nm   	|
|  nominal total traction force 	| 725 N   	|
|  maximum total traction force 	| 1511 N   	|
|               hill climb grade 	| 96% (44°)	|
| climb grade with 50 kg cargo 	    | 90% (42°) |
| climb grade with 80 kg cargo 	    | 60% (31°)	|
|          hill grade traversal 	| 80% (39°)	|

**International Protection Rating**

The platform is offered in two variants of the protection class. The basic variant is dedicated for moderate indoor and outdoor conditions with a rating of IP54. The upgraded variant is dedicated for extremely demanding work environment with a rating of IP66. Sales details such as price, lead time and other conditions available in the store.

Specification of given ratings:
|Class|Solid|Fluid|
|:---:|:---:|---|
|**IP54**|dust protected|protection against splashes of water from any direction|
|**IP66**|dust-tight|protection against strong water jets (100 l / min) poured on the housing from any side|

### Components ###


| Component | Quantity | Description |
| --- | --- | --- |
| Internal computer | 1 | Raspberry Pi 4B with Broadcom BCM2711 processor, quad-core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz and 4GB LPDDR4 RAM. Used to manage all the basic functions of a mobile platform. |
| On-board computer * | 1 | **Intel NUC10i7FNK** or **ADLINK Vizi-AI** or **HP Z2 Mini Workstation with Nvidia graphics card** |
| Router | 1 | Teltonika RUTX11 - Dual-band (2.4 GHz/5 GHz), Access Point / Client Mode, 4G LTE CAT 6 dual SIM, Bluetooth 4.0 LE, GNSS (GPS, GLONASS, BeiDou, Galileo and QZSS) - This device ensures reliable communication between internal and external components of the robot system. Fast LTE communication and dual-band WiFi allow you to maintain communication with the robot. [More details](https://teltonika-networks.com/product/rutx11/). |
| Antenna | 2 | Dual-band (2.4 GHz/5 GHz) placed on the rear of the robot. See all [available options]( https://husarion.com/docs/assets/pdf/available_options_revA.pdf "Husarion Panther available extension options") (soon), chapter Communication.|
| Inertial navigation system | 1 | PhidgetSpatial 3/3/3 Basic (3-axis compass, a 3-axis gyroscope, and a 3-axis accelerometer) [More details](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1025). |
| Front and rear lights | 2 | Signal lighting made of 48 pcs. APA102C LED chips build into an aluminum profile on the robot's bumpers. |
| Brushless Motor with planetary gearbox | 4 | 80PMB800K.80RBL-100 - Drive implemented on 4 durable motors 473 watts of power (900 W instantaneous power) each and planetary gears with a maximum torque of 60Nm allows the robot to move at a speed of 2 m/s even uphill with a slope of 40% with a load of 50 kg. |
| Additional kits** |  | Together with the robot, you can get an integrated lidar, depth camera, manipulator and more. See all [available options]( https://husarion.com/docs/assets/pdf/available_options_revA.pdf "Husarion Panther available extension options") (soon)|

> **Note:** *By default, there is only up to one on-board computer in the robot. For detailed information please check System installation and read more about [available options]( https://husarion.com/docs/assets/pdf/available_options_revA.pdf "Husarion Panther available extension options") (soon), chapter User computer.

> **Note:** **Most of external modules are attached to the profiles on the top of the platform. [More details](/manuals/panther/#mounting-rails).

#### Communication ####
**available as standard**

* Ethernet
* USB
* Wi-Fi (2.4GHz & 5GHz)


**possible to extend**

* LTE
* GPS
* CAN
* RS232
* RS485
* I2C
* SPI

See all [available options]( https://husarion.com/docs/assets/pdf/available_options_revA.pdf "Husarion Panther available extension options") (soon), chapter Communication.

### Block diagram ###

Graphic representation of Panther components and connections between them. A full, more detailed version of the block diagram can be downloaded [here](https://files.husarion.com/panther/schematic_block_diagram.pdf).

![Block diagram](/img/Panther/simplified_block_diagram.png "Block diagram")

### Power Switch and Emergency button ###

The robot is equipped with a three-position Main switch and Emergency push button.

| Main switch position |  Name of position 	| LED state |              Power state              |
| :------------------: | :----------------:	| :-------: | :-----------------------------------: |
|        left          |         Off        |     Off   |              Turned off               |
|       center         |       Stage 1      |     On    | The robot turned on except the motors |
|        right         |       Stage 2      |     On    |  The robot fully ready for operation  |

Pushing the safety switch completely cuts off the power to the device.

> **Note:** Cutting off the computer's power may cause data loss.

![Power switch positions](/img/Panther/power_switch.png "Power switch positions")

### Power supply ###

The Panther is equipped with a set of battery cells in Lithium-Ion technology with a rated voltage of 36V and 20Ah, which gives it 740Wh of energy to use for calculations and move around in demanding terrain for about 3.5 hours. Moving the robot in a friendly terrain allows for a significant extension of the robot's working time up to 8 hours (standby time up to 40 hours). You can check more specific information about Panther power consumption [here]( https://husarion.com/docs/assets/pdf/panther_power_consumption_and_run_time.pdf "Husarion Panther power consumption and run time").

Low battery level will be indicated by a change in the bumper light - both lights will fade in and out orange to indicate that the battery voltage has dropped below 35 volts. Read more in the [Bumpers and signal lights](#bumpers-and-signal-lights) section.

To meet the user's needs, the robot is equipped with 9 high-power electrical connectors that are able to provide a total of 505W of power to the user's devices.
The supply voltages available on the user Power panel are 5V with a total current limitation up to 15A (3x female XT60), 12V limited to 20A (3x female XT60) and 19V limited to 10A (3x female XT60).

![User Panel - Power Supply](/img/Panther/UserPanel_PowerSupply.png "User Panel - Power Supply")

> **Note:** One of 19V output is used by NUC and one of 12V output is used by router.


|          Name          	|          Value            |
|:----------------------:	|:------------------------: |
|       Battery capacity 	| 740Wh  	                |
|                Runtime 	| 3.5 h  	                |
|     Total output power 	| 1 kW  	                |
|     Maximum peak power 	| 1.8 kW 	                |
|     Max power for user use* 	| 5V@15A, 12V@20A, 19V@10A  |
|     Total power for user use* 	| 360W  |


> **Note:** *Each of the voltage sources has an independent overcurrent switch, but the total power consumed by the devices plugged into the Power panel **cannot exceed 360W**.

### Charging Panther ###

In the set with the robot, we provide a dedicated 42V @ 5A charger, which the robot will charge to 80% in 4 hours, and to 100% in 7 hours. The mains-operated charger is connected directly to the robot's charging connector on its housing.

> **Warning!** It is highly recommended not to use the robot during the charging process! When the charger is connected to the robot, the robot should be turned off completely.

> **Warning!** Please do not leave charger connected to the robot after the charging process is completed (the red LED and charger fan will turn off).

### Mounting rails ###

Sensors, constructions and payload can be attached to the profiles on top of the robot. 

![Rails on top of the Robot](/img/Panther/top_rails_v1.0.png "Rails on top of the Robot")

The profiles used are aluminum V-slot 2020 profiles. The best way to attach the elements to them is to use mounting elements dedicated to this type of profiles, such as T-nuts, fittings and angles.

![V-slot aluminum extrusion](/img/Panther/v-slot_profile.png "Block diagram")

These profiles are fixed to the robot with four 8mmx40mm quick release pins with ball lock. 

![Top rails fixing](/img/Panther/top_rails_mounting_v1.0.png "Top rails fixing")

To enable access to user space, top rails can be pivoted by pulling out 2x quick release pin in both front and back direction. Maximum opening angle is 160 degrees with built-in stop.
![Top rails open](/img/Panther/top_rails_open_v1.0.png "Top rails open") 
For more useful information in the field of mechanics, please see the document [Panther Overall Dimensions]( https://husarion.com/docs/assets/pdf/Husarion_Panther_overall_dimensions.pdf "Husarion Panther Overall Dimensions") and chapter [CAD models](/manuals/panther/#cad-models).

> **Note** The presence of railings has no effect on the water and dust resistance of the robot.

### Bumpers and signal lights ###

The robot's bumpers are made of a profile with the same cross-section as the railings. They are 0.5 meters wide and, in addition to buffer shock and reducing potential damage when the robot collides with an obstacle at high speed, additional sensors can be mounted on them. By default, the signaling lighting is mounted on each bumper in the form of an aluminum profiles with 46 programmable RGB LEDs.

These lights may be widely used - to indicate the status of the robot, the direction of movement or the intention to change direction, warn about low battery or other detected errors, signal the status of the charging process or even for illuminate the area in front of the robot. More details regarding lights and their control is available in [ROS API](#ros-api) section.


### Access to the interior ###

The robot's volume has been divided into three parts.
The central space is dedicated to the user's components and electronics. Here, by default, the Inter NUC computer, RUTX11 router and the robot's battery are located. Two user panels has been led to this space. A panel distributing electric power for the user:
![User Panel - Power Supply](/img/Panther/UserPanel_PowerSupply.png "User Panel - Power Supply")
And a panel for communication with the rest of the robot - as standard it is an Ethernet connector to the internal SBC):
![User Panel - Communication Port](/img/Panther/UserPanel_CommunicationPort.png "User Panel - Communication Port")
The front and rear spaces are occupied by motors and built-in electronics. For these spaces it is usually not needed to access by the user. Opening these spaces is mainly used for service work.

To access the components inside the user space, pivot the top rails by pulling 2x quick release pin and then unscrew Cover (18x DIN912 M5x12).
![Cover fixing](/img/Panther/access_to_user_space_v1.0.png "Cover fixing")

To access the components in service space, pivot the top rails by pulling 2x quick release pin and then unscrew (19x DIN912 M5x12).
![Deck fixing](/img/Panther/access_to_service_space_v1.0.png "[Deck fixing")

The bolts must be re-tightened with a torque of 4-5 Nm.
> **Warning!** A necessary condition to meet the protection rate for external conditions and the faultless operation of the robot is lack of foreign objects on the seal.

> **Warning!** A necessary condition to meet the protection rate for external conditions and the faultless operation of the robot is the correct tightening of all screws fixing the Cover and both Decks!

### Space inside the robot ###

Inside the robot, there is a dedicated volume of approximately 14.5 liters (3.8 gallons) for the user's equipment and devices. This part of the robot has the same water and dust tightness class as the robot. By default, there is the Intel NUC on-board computer and the RUTx11 router responsible for both the communication of the robot with the inner world and the connection of computers and sensors within the robot.

![User's space](/img/Panther/UserSpaceDrawning.png "[User's space")

### CAD models ###

To facilitate the work with the project based on Panther platform, we have prepared CAD models for download in three extension formats:

<a href="https://husarion.com/docs/assets/models/Panther_v1.0.step.zip" download target="_blank">STEP</a>
<br />
<a href="https://husarion.com/docs/assets/models/Panther_v1.0.iges.zip" download target="_blank">IGES</a>
<br />
<a href="https://husarion.com/docs/assets/models/Panther_v1.0.stl.zip" download target="_blank">STL</a>


## Software guide ##

Panther robot is equipped with the Raspberry Pi 4 SBC with custom OS based on Ubuntu 20.04 and contains all components needed to start working with ROS immediately. The microSD card with OS for the Raspberry Pi is included with each Panther robot. The OS contains software drivers for all components and has been modified to make the file system insensitive to sudden power cuts.

### ROS API ###

Below are topics and services available in Panther:

| Topic | Message type | Direction | Node |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- | --- |
| `/joint_states`| `sensor_msgs/JointState` | publisher | `/panther_driver` | Wheels angular position [encoder pulses], speed [encoder pulses per second] and motor current [Amperes] oredered as [Front left, Front right, Rear left, Rear right]|
| `/battery` | `sensor_msgs/BatteryState` | publisher | `/panther_driver` | Battery voltage |
| `/pose` | `geometry_msgs/Pose` | publisher | `/panther_driver` | Position based on encoders |
| `/cmd_vel` | `geometry_msgs/Twist` | subscriber | `/panther_driver` | Velocity commands |
| `/set_panther_lights` | `panther_lights/LightsMessage` | service | `/panther_lights` | Control the front and rear lights |
| `/imu/data` | `sensor_msgs/Imu` | publisher | `/imu_manager` | Publishes imu data |


GPS API: 

GPS data in [NMEA](https://en.wikipedia.org/wiki/NMEA_0183) format is forwarded to main computer IP address (Intel NUC/ Vizi-AI) at port 5000, typically it is `10.15.20.3:5000`. 

Data frequency is 1Hz and can be interacted ether with GPSD daemon (`gpsd -N -D 5 udp://10.15.20.3:5000`) or directly with [ROS package](https://github.com/ros-drivers/nmea_navsat_driver/tree/decode-udp-line) redirecting signal to ROS topic. 

After cloning [this package](https://github.com/ros-drivers/nmea_navsat_driver/tree/decode-udp-line) [IMPORTANT: use branch `decode-udp-line`] into your workspace (`~/husarion_ws/src`) use following command to run:

```
rosrun nmea_navsat_driver nmea_socket_driver _port:=5000 _local_ip:=10.15.20.3
```

or use launch file:

```
<launch>
  <arg name="port" default="5000" />
  <arg name="local_ip" default="10.15.20.3" />

  <node pkg="nmea_navsat_driver" type="nmea_socket_driver" name="nmea_socket_driver_node" output="screen">
    <param name="port" type="int" value="$(arg port)"/>
    <param name="local_ip" type="str" value="$(arg local_ip)" />
  </node>
</launch>
```

You should be able to see data on `/fix` topic (`rostopic echo /fix`). 

Lights pattern could be set by calling appropriate message to `/set_panther_lights` service, to change animation to `BLINKER_LEFT`:
```
rosservice call /set_panther_lights "animation: 1
custom_color: ''"
```

To change animation to `BLINKER_LEFT` with custom colors (front is green, rear is red):
```
rosservice call /set_panther_lights "animation: 1
custom_color: '0x00FF00 0xFF0000'"
```
More details regarding lights control could be found in [`panther_lights` documentation](https://github.com/husarion/panther_lights).

More details regarding driver could be found in [`panther_driver` documentation](https://github.com/husarion/panther_driver).

### Disable autostart ROS nodes ###

Brand new Panther launches some ROS nodes, you can disable them just by sending systemctl commands.

_ROSCORE_ :

By default roscore starts at main computer and can be disabled by logging (default `ssh husarion@10.15.20.3` pass:`husarion`) and executing following command:

```systemctl disable roscore.service```

_ROUTE ADMIN PANEL_ 

By default route admin panel starts at main computer and can be disabled by logging (default `ssh husarion@10.15.20.3` pass:`husarion`) and executing following command:

```systemctl disable route_admin_panel.service```

_PANTHER DRIVER_ 

Panther driver starts at RPI SBC and it's responsible for controlling motors, and also sync time from main computer it's not recommended to disable this because you can end up with timestamp problems. Instead we recommend to customize [launch file](https://github.com/husarion/panther_driver/blob/main/launch/driver.launch) by setting appropriate arguments. If you still want to disable this use:

```sudo systemctl disable launch_driver.service```


#### External documentation ####

 - Slamtec RpLidar scanner API is documented in [rplidar repository](https://github.com/Slamtec/rplidar_ros)

### Joystick control ###

The Raspberry Pi SBC has a preinstalled WEBUI with simple joystick. The joystick allows user to issue simple motion commands for the robot.
To use the joystick, open `RASPBERRY_PI_IP_ADDRESS:8000`.

## System installation ##

 In some cases you will need to restore Panther's system to its default settings:
 - in case of accidental damage of the system,
 - to update the OS,
 - to clear all user changes and restore factory settings.

### Raspberry Pi 4 ###
 The reinstallation procedure for on-board RPI:

1. Extract SD card from Raspberry Pi SBC by pushing card carefully until it is released back by card holder, thel pull it out. In order to find SD card slot, you will need to disassemble part of the top cover.
2. Download image for Raspberry Pi from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/production_images/ros-noetic-rpi-2021-03-29.img.xz).
3. Extract downloaded image (For this process we recommend using [unxz](https://linux.die.net/man/1/unxz) tool).
4. Flash the extracted image onto SD card (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
 - If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 
 - Download [Etcher](https://www.balena.io/etcher/) and install it.
 - Connect an SD card reader with the SD card inside.
 - Open Etcher and select from your hard drive .img file that you extracted.
 - Select the SD card you wish to write your image to.
 - Review your selections and click 'Flash!' to begin writing data to the SD card.
5. Insert SD card back to Raspberry Pi

### Intel NUC / HP G2 ###
To install system on *Intel NUC* or *HP G2*, you can download created ready to use Ubuntu20 image. 

1. Download image from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/production_images/ros-noetic-x64-2021-04-01.iso.xz).
2. Create bootable pendrive with [Etcher](https://www.balena.io/etcher/).
3. Insert pendrive into one of Panther's USB ports and install by selecting appropriate option during boot.

Optional you can install clear image of your favorite Linux distro and use [husarion docker](https://github.com/adamkrawczyk/husarion_docker)

### Vizi-AI ###
To install system on Vizi-AI 

**How to install and use**

**Install**

1. [Download](https://vizi-ai-ppa.s3.eu-west-2.amazonaws.com/Win-mac/Vizi-AI.xz) image - official instruction is [here](https://www.goto50.ai/re-installing-or-upgrading-the-vizi-ai-sd-card-image/)
2. Flash the extracted image onto SD card (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
3. When using Vizi-AI:
    - Connect your Vizi to Internet, insert SD card, plug in keyboard and power on.
    - Login using L:`root`, P:`root`
    - Enable ssh with following commands:
    
    ```
    sed -i 's/#PermitRootLogin prohibit-password /PermitRootLogin yes/' /etc/ssh/sshd_config
    systemctl enable ssh
    systemctl start ssh
    ip a # get your IP address for connection (should be under br-enp6s0)(#should be 10.15.20.3)
    ```
    - Upload file [set_vizi-ai.sh](https://github.com/adamkrawczyk/husarion_docker/blob/main/panther_system_ros1/set_vizi-ai.sh) (scp set_vizi-ai.sh root@10.15.20.3:/root)
    - Execute file `./set_vizi-ai.sh` - This will create user husarion with password husarion set your environment and also download container.
4. When not using script download image 

```
docker pull khasreto/panther_system_ros1:latest
```

5. Run image 

```
docker run  --net=host -e ROS_MASTER_URI -e ROS_IP -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --privileged --name panther_system_ros1 khasreto/panther_system_ros1:latest
```

The Code Explained:

`--net=host` - start a container in the host network, from the perspective of the network look like they are running on the host itself.

`-e ROS_MASTER_URI -e ROS_IP` - set environment variables (should be inside ~/.bashrc).

`-it` - interactive terminal.

`-v /tmp/.X11-unix:/tmp/.X11-unix` - Set the X11 server communication.

`-v /dev:/dev` - Gives permissions to use /dev/*, needed to launch for example Lidar.

`--privileged` - Give extended privileges to this container, container can not only access to all hosts devices but also use most of host computer’s kernel functions.

`--name panther_system_ros1` - Name of container (the one used for future launches and container identification).

`khasreto/panther_system_ros1:latest` - Name of image.

6. If you want to autostart:
```
touch /etc/systemd/system/docker-panther-system.service
```

To file /etc/systemd/system/docker-panther-system.service paste following code:
```
[Unit]
Description=Panther System ROS1 noetic Container
Requires=docker.service
After=docker.service NetworkManager.service time-sync.target

[Service]
Restart=always
ExecStart=/usr/bin/docker start -a panther_system_ros1
ExecStop=/usr/bin/docker stop -t 2 panther_system_ros1

[Install]
WantedBy=local.target
```

Enable service

sudo systemctl enable docker-panther-system.service


**Usage**

- If you have made an autostart then to access container use following command:

    `docker exec -it panther_system_ros1 /bin/bash`

- In case you haven't made an autostart use:

    `docker start -i panther_system_ros1`

    Then 

    `docker exec -it panther_system_ros1 /bin/bash`
#### Launching navigation example

The user space PC comes with preinstalled Ubuntu 20.04 and ROS, the same as the Raspberry Pi SBC. PC also has a [route_admin_panel](https://github.com/husarion/route_admin_panel/) as an example application.

NOTE: If using Docker run every command inside container

To start the RAP:

PC:

```
roscore
roslaunch route_admin_panel demo_panther_classic.launch # chose between classic mix and mecanum depending of your wheel configuration
```

RPI:
```
roslaunch panther_driver driver.launch 
```

Then open in browser: 

```
PANTHER_IP_ADDRESS:8000 #default 10.15.20.3:8000
```

You should see interface like below:

![RouteAdminPanelScreenshot](/img/software/route-admin-panel.png)

## Network ##

### Overview ###

Panther is equipped with a RUTX11 router running open-source firmware OpenWRT, which provide following interfaces:

**Ethernet**

* 1 x WAN 10/100/1000 Mbps (by default configured as a LAN port)
* 3 x LAN 10/100/1000 Mbps 
* 3 ports are available for user equipment

**Wireless**

* Two radios (2.4 GHz and 5 GHz)
* Support for 802.11ac (Wi-Fi 5) with link rate up to 867Mbps, fast roaming with 802.11r
* Works as access point (AP) and/or as a client/station (STA)
* External antennas with 2dBi gain (standard equipment of 'Pth10')

**Cellular**

* Dual-Sim with fail-over
* LTE (4G) Cat 6
* Two external antennas with 3dBi gain (equipped as an option 'Ant01')

**GNSS**

* Support for GPS, GLONASS, Galileo and BeiDou
* Integrated into ROS
* External antenna (equipped as an option 'Ant01')

Each form of connectivity can be part of automatic WAN fail-over in order to provide continuous connection to external services.
More information is available on manufacturer [site](https://wiki.teltonika-networks.com/view/RUTX11_Manual). Some RUTX11 configuration scripts are available on [our GitHub repository](https://github.com/husarion/panther_rutx11).
### Connecting to Panther's hotspot ###

Panther provides hotspot with default SSID `Panther_XXXX` and `Panther_5G_XXXX` for 2.4 GHz and 5 GHz band respectively with password `husarion`, where `XXXX` is unique S/N of your Panther.

### Connecting Panther to Wi-Fi ###

Panther can be connected Wi-Fi on 2.4GHz or 5GHz band. It will be used as WAN source and be prioritized over cellular connection. Single radio can act simultaneously as AP (access point) and STA (client).

> **Note:**: We advise to use 2.4GHz radio as a up-link.

By default Panther scan for available networks and connect to first one provided in configuration file. In case of low signal level or lost of signal next one on list will chosen (if available). This behavior can be modified by user.

#### Connecting to 2.4 GHz Wi-Fi ####

Open a Linux terminal in your laptop that is connected to the hotspot provided by Panther

```$ ssh husarion@10.15.20.3``` with password `husarion`
To edit network configuration edit file ```~/panther_rutx11/config.json```:

``` nano panther_rutx11/config.json```

Edit section named `wifi_client` filling in your SSID and matching password. If you want to connect to multiple networks duplicate whole block, Panther will try to connect to them in given order:

 ```
        {
            "radio":"0",
            "ssid":"SSID_of_your_network",
            "password":"password_to_your_network",
            "encryption":"psk2"
        }
```

Save BY pressing `Ctrl+O` and exit by `Ctrl+X`. To apply new settings execute python script by command:

``` panther_rutx11/setup.py```

When Panther connect to our network message `Success` will be shown. For explanation of possible configuration options go to [our GitHub repository](https://github.com/husarion/panther_rutx11)

#### Connecting to 5 GHz Wi-Fi ####

Due to limitation of Wi-Fi chipset it is not possible to scan for available networks, while providing AP on 5 GHz interface. Its advised to use 2.4 GHz for WAN uplink. For more information go to documentation on [our GitHub repository](https://github.com/husarion/panther_rutx11)

### Access to router WebUI ###

Further configuration can be done through WebUI, which is available under `10.15.20.1` address. Login with username `admin` and password `Husarion1`. Consult [Teltonika RUTX11 manual](https://wiki.teltonika-networks.com/view/RUTX11_Manual) for more information.
> **Note:** It is advised to change default password.

### Resetting router to default settings ###

In case of misconfiguration it is possible to reset router to working default settings (as shipped to you). To do it press and hold reset button on powered on RUTX11 for at least six seconds. Signal strength LEDs indicate elapsed time. After all five LEDs are lit up, reset button can be released. After automatic restart router is ready to be used again.

## Docs and links ##
All helpful documents and links in one place:

* [Safety instructions]( https://husarion.com/docs/assets/pdf/Husarion_Panther_safety_instructions.pdf "Husarion Panther safety instructions") - to avoid malfunction or damage your Panther please read this safety manual before use
* [Panther schematic block diagram]( https://husarion.com/docs/assets/pdf/schematic_block_diagram.pdf "Panther schematic block diagram") - basic robot components and connections between them
* [Panther Overall Dimensions]( https://husarion.com/docs/assets/pdf/Husarion_Panther_overall_dimensions.pdf "Husarion Panther Overall Dimensions") - three basic projections of the platform in all wheel options
* [Panther power consumption and run time]( https://husarion.com/docs/assets/pdf/panther_power_consumption_and_run_time.pdf "Husarion Panther power consumption and run time") - description of Panther power consumption and run time in different working conditions
* [Teltonika RUTX11 manual](https://wiki.teltonika-networks.com/view/RUTX11_Manual)
* [Teltonika RUTX11 datasheet](https://teltonika-networks.com/downloads/en/rutx11/RUTX11-Datasheet.pdf)


