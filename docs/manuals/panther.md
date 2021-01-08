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

Autonomous, mobile robot platform dedicated for outdoor environment. Compliant with IP54 rate of protection. Depending on the use-case it can be equipped with a robot arm, LIDAR, RGB-D camera, GPS, UWB and other additional equipment. It can be used in various areas of application such as agriculture, rescue, inspection and many more.


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

## Quick Start ##
Here is a very basic demo allowing you to use your Panther the first time without coding:

Rotate emergency push button + set power switch to right (it is 3 pose switch, set to the 3rd pose), and then wait for Wi-Fi to come up:


> **SSID:** `Panther_<serial_number>`
>
> **PASS:** `husarion`

Connect to Wi-Fi and open **WEBUI ROS JOYSTICK** [`10.15.20.2:8000`] that allows you to manually control your robot. You can also test **Route Admin Panel** that will be available under [`10.15.20.3:8000`] address (default addresses and ports for Panther). 

When the robot is ready to work you should see it's lights flashing.

[Route Admin Panel](https://husarion.com/software/route-admin-panel/) (RAP) is a preinstalled, open-source web user interface available on the Panther. By using RAP you can test autonomous drive of your robot - you can define navigation points and send orders to Panther to visit them. Before using RAP, at first you need to create a basic map of the environment you are going to test the robot - you can do it manually by a web-joystick available under [`10.15.20.2:8000`].


> **WARNING:** RAP is not a production-ready software, **that's a basic demo you use on your own risk**. Especially some obstacles in your working environment might be invisible for sensors in your configuration. Feel free to modify this demo code. It is open source and available on Husarion's github https://github.com/husarion/route_admin_panel .
> Also be careful while using a WEBUI ROS JOYSTICK. During testing Panther, especially during a first run, be prepared to push the emergency button rapidly if needed. Panther is quite heavy robot with high power motors, so be carefull while working with it. 


## Remote Access
You can access your Panther over the Internet, from any place in the world, thanks to Husarnet VPN service (husarnet.com). Husarnet is preinstalled on the Panther. To access your robot from any place in the world, just follow these steps:

Create an account at https://app.husarnet.com/ and click **[Create network]** button. Go to the newly created Husarnet network and click **[Add element]** button. Go to a **[join code]** tab and copy your join code, it should look like this:

`fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxxxxxxxxxx`

Open a Linux terminal in your laptop that is connected to the hotspot provided by Panther

```$ ssh husarion@10.15.20.2``` with password `husarion`

To connect Panther to Wi-Fi on 2.4GHz named `MyNetwork` with password `MyPassword` execute script:

 `~/panther_rutx11/setup.sh -s MyNetwork -p MyPassword -r 0`

After message `Network added` exit terminal session by command `exit`

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
|       protection index 	| IP54  	|

**Traction**

|              Name             	|  Value  	|
|:-----------------------------:	|:-------:	|
|                     max speed 	| 2 m/s   	|
|     maximum carrying capacity     | 80 kg     |
|          nominal shaft torque 	| 17.4 Nm 	|
|          maximum shaft torque 	| 60 Nm   	|
|  nominal total traction force 	| 365 N   	|
|  maximum total traction force 	| 630 N   	|
|               hil climb grade 	| 96%     	|
| climb grade with 50 kgs cargo 	| 40%     	|
| climb grade with 80 kgs cargo 	| 30%     	|
|          hill grade traversal 	| 80%     	|


### Components ###

#### Standard ####

| Component | Quantity | Description |
| --- | --- | --- |
| Internal computer | 1 | Raspberry Pi 4B with Broadcom BCM2711 processor, quad-core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz and 4GB LPDDR4 RAM. Used to manage all the basic functions of a mobile platform. |
| On-board computer * | 1 | **Intel NUC10i7FNK** / **ADLINK Vizi-AI** / **HP Z2 Mini Workstation** with **Nvidia graphics card** |
| Router | 1 | Teltonika RUTX11 - Dual-band (2.4 GHz/5 GHz), Access Point / Client Mode, 4G LTE CAT 6 dual SIM, Bluetooth 4.0 LE, GNSS (GPS, GLONASS, BeiDou, Galileo and QZSS) - This device ensures reliable communication between internal and external components of the robot system. Fast LTE communication and dual-band WiFi allow you to maintain communication with the robot. [More details](https://teltonika-networks.com/product/rutx11/). |
| Inertial navigation system | 1 | PhidgetSpatial 3/3/3 Basic (3-axis compass, a 3-axis gyroscope, and a 3-axis accelerometer) [More details](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1025). |
| Front and rear lights | 2 | Signal lighting made of 48 pcs. APA102C LED chips build into an aluminum profile on the robot's bumpers. |
| Brushless Motor with planetary gearbox | 4 | 80PMB800K.80RBL-100 - Drive implemented on 4 durable motors 473 watts of power (900W instantaneous power) each and planetary gears with a maximum torque of 60Nm allows the robot to move at a speed of 2 m/s even uphill with a slope of 40% with a load of 50 kg. |

> **Note:** *By default, there is only one on-board computer in the robot. For detailed information please check System installation

#### External modules ####

* headlights,
* RGBD camera,
* thermal camera,
* manipulator,
* ect.

Most of external modules are attached to the profiles on the top of the platform. [More details](https://husarion.com/manuals/panther/#mounting-rails).

#### Communication ####
**available as standard**

* Ethernet,
* USB,
* Wi-Fi (2.4GHz & 5GHz),
* LTE,
* GPS.


**possible to extend**

* CAN,
* RS232,
* RS485,
* I2C,
* SPI

### Block diagram ###

Graphic representation of Panther components and connections between them. A full, more detailed version of the block diagram can be downloaded [here](https://files.husarion.com/panther/schematic_block_diagram.pdf).

![Block diagram](/docs/assets/img/Panther/simplified_block_diagram.png "Block diagram")

### Power Switch and Emergency button ###

The robot is equipped with a three-position Main switch and Emergency push button.

| Main switch position |  Name of position 	| LED state |              Power state              |
| :------------------: | :----------------:	| :-------: | :-----------------------------------: |
|        left          |         Off        |     Off   |              Turned off               |
|       center         |       Stage 1      |     On    | The robot turned on except the motors |
|        right         |       Stage 2      |     On    |  The robot fully ready for operation  |

Pushing the safety switch completely cuts off the power to the device.

> **Note:** Cutting off the computer's power may cause data loss.

![Power switch positions](/docs/assets/img/Panther/power_switch.png "Power switch positions")

### Power supply ###

The Panther is equipped with a set of battery cells in Lithium-Ion technology with a rated voltage of 36V and 20Ah, which gives it 740Wh of energy to use for calculations and move around in demanding terrain for about 3.5 hours. Moving the robot in a friendly terrain allows for a significant extension of the robot's working time up to 8 hours (standby time up to 40 hours).

Low battery level will be indicated by a change in the bumper light - both lights will fade in and out orange to indicate that the battery voltage has dropped below 35 volts. Read more in the [Bumpers and signal lights](#bumpers-and-signal-lights) section.

To meet the user's needs, the robot is equipped with 9 high-power electrical connectors that are able to provide a total of 505W of power to the user's devices.
The supply voltages available on the user Power panel are 5V with a total current limitation up to 15A (3x female XT60), 12V limited to 20A (3x female XT60) and 19V limited to 10A (3x female XT60).

![User Panel - Power Supply](/docs/assets/img/Panther/UserPanel_PowerSupply.png "User Panel - Power Supply")

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

![Rails on top of the Robot](/docs/assets/img/Panther/top_rail.png "Rails on top of the Robot")

The profiles used are aluminum V-slot 2020 profiles. The best way to attach the elements to them is to use mounting elements dedicated to this type of profiles, such as T-nuts, fittings and angles.

![V-slot aluminium extrusion](/docs/assets/img/Panther/v-slot_profile.png "Block diagram")

These profiles are fixed to the robot with four bolts DIN912 M8x40.

![Top rails fixing](/docs/assets/img/Panther/top_rail_M8.png "Top rails fixing")

For more useful information in the field of mechanics, please see the document [Overall dimensions](https://files.husarion.com/panther/external_dimentions.pdf "Overall dimensions") and chapter [CAD models](https://husarion.com/manuals/panther/#cad-models).

> **Note** The presence of railings has no effect on the water and dust resistance of the robot.

### Bumpers and signal lights ###

The robot's bumbers are made of a profile with the same cross-section as the railings. They are 0.5 meters wide and, in addition to buffer shock and reducing potential damage when the robot collides with an obstacle at high speed, additional sensors can be mounted on them. By default, the signaling lighting is mounted on each bumper in the form of an aluminum profiles with 46 programmable RGB LEDs.

These lights may be widely used - to indicate the status of the robot, the direction of movement or the intention to change direction, warn about low battery or other detected errors, signal the status of the charging process or even for illuminate the area in front of the robot. More details regarding lights and their control is available in [ROS API](#ros-api) section.


### Access to the interior ###

The robot's volume has been divided into three parts.
The central space is dedicated to the user's components and electronics. Here, by default, the Inter NUC computer, RUTX11 router and the robot's battery are located. Two user panels has been led to this space. A panel distributing electric power for the user:
![User Panel - Power Supply](/docs/assets/img/Panther/UserPanel_PowerSupply.png "User Panel - Power Supply")
And a panel for communication with the rest of the robot - as standard it is an Ethernet connector to the internal SBC):
![User Panel - Communication Port](/docs/assets/img/Panther/UserPanel_CommunicationPort.png "User Panel - Communication Port")
The front and rear spaces are occupied by motors and built-in electronics. For these spaces it is usually not needed to access by the user. Opening these spaces is mainly used for service work.

To access the components inside the user space, unscrew the top rails (4x DIN912 M8x40) and then unscrew Cover (18x DIN912 M5x12).
![Cover fixing](/docs/assets/img/Panther/cover_M5.png "Cover fixing")

To access the components in service space, unscrew the top rails and then (19x DIN912 M5x12).
![Deck fixing](/docs/assets/img/Panther/deck_M5.png "[Deck fixing")

The bolts must be re-tightened with a torque of 4-5 Nm.

> **Warning!** A necessary condition to meet the protection rate for external conditions and the faultless operation of the robot is the correct tightening of all screws fixing the Cover and both Decks!

### Space inside the robot ###

Inside the robot, there is a dedicated volume of approximately 14.5 liters (3.8 gallons) for the user's equipment and devices. This part of the robot has the same water and dust tightness class as the robot. By default, there is the Intel NUC on-board computer and the RUTx11 router responsible for both the communication of the robot with the inner world and the connection of computers and sensors within the robot.

![User's space](/docs/assets/img/Panther/UserSpaceDrawning.png "[User's space")

### CAD models ###

To facilitate the work with the project based on Panther platform, we have prepared CAD models for download in three extension formats:

* [STEP](/docs/assets/models/Panther_v0.2.step.zip "STEP model")
* [IGES](/docs/assets/models/Panther_v0.2.iges.zip "IGES model")
* [STL](/docs/assets/models/Panther_v0.2.stl.zip "STL model")

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

Data frequency is 1Hz and can be interacted ether with GPSD daemon (`gpsd -N -D 5 udp://10.15.20.3:5000`) or directly with [ROS package](https://github.com/adamkrawczyk/nmea_navsat_driver) redirecting signal to ROS topic. 

After cloning [this package](https://github.com/adamkrawczyk/nmea_navsat_driver) into your workspace (`~/husarion_ws/src`) use following command to run:

```
rosrun nmea_navsat_driver nmea_socket_driver _port:=udp://10.15.20.3:5000
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
 - in case of accidential damage of the system,
 - to update the OS,
 - to clear all user changes and restore factory settings.

### Raspberry Pi 4 ###
 The reinstallation procedure for on-board RPI:

1. Extract SD card from Raspberry Pi SBC by pushing card carefully until it is released back by card holder, thel pull it out. In order to find SD card slot, you will need to disassemble part of the top cover.
2. Download image for Raspberry Pi from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/production_images/ros-noetic-rpi-2020-10-15.img.xz).
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

1. Download image from [here](https://husarion-files.s3-eu-west-1.amazonaws.com/production_images/ros-noetic-x64-2020-10-13.iso).
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

![RouteAdminPanelScreenshot](/docs/assets/img/software/route-admin-panel.png)

## Network ##

### Overview ###

Panther is equipped with a RUTX11 router running open-source firmware OpenWRT, which provide following interfaces:

**Ehernet**

* 1 x WAN 10/100/1000Mbps (by default configured as a LAN port)
* 3 x LAN 10/100/1000Mbps 
* 2 ports are available for user equipment

**Wireless**

* Two radios (2.4GHz and 5GHz)
* Support for 802.11ac (Wi-Fi 5) with link rate up to 867Mbps, fast roaming with 802.11r
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
More information is available on manufacturer [site](https://wiki.teltonika-networks.com/view/RUTX11_Manual). Some RUTX11 configuration scripts are available on [our GitHub repository](https://github.com/husarion/panther_rutx11).

### Connecting Panther to Wi-Fi ###

Panther can be connected Wi-Fi on 2.4GHz or 5GHz band. It will be used as WAN source and be prioritized over cellular connection. Single radio can act simultaneously as AP (access point) and STA (client).

#### Connecting to 2.4GHz Wi-Fi ####

Open a linux terminal in your laptop that is connected to the hotspot provided by Panther

```$ ssh husarion@10.15.20.2``` with password `husarion`
To connect Panther to Wi-Fi on 2.4GHz named `MyNetwork` with password `MyPassword` execute script:

`~/panther_rutx11/setup.sh -s MyNetwork -p MyPassword -r 0`

After message `Network added` exit terminal session by command `exit`.

#### Connecting to 5GHz Wi-Fi ####

Due to limitation of Wi-Fi chipset it is not possible to connect to 5GHz network without its brief shutdown. Its advised to connect to Panther with 2.4GHz Wi-Fi when executing commands below.

```$ ssh husarion@10.15.20.2``` with password `husarion`
To connect Panther to Wi-Fi on 2.4GHz named `MyNetwork` with password `MyPassword` execute script:

 `~/panther_rutx11/setup.sh -s MyNetwork -p MyPassword -r 1`

After message `Network added` exit terminal session by command `exit`.

### Access to router WebUI ###

Further configuration can be done through WebUI, which is available under `10.15.20.1` address. Login with username `admin` and password `Husarion1`. Consult [Teltonika RUTX11 manual](https://wiki.teltonika-networks.com/view/RUTX11_Manual) for more information.
> **Note:** It is advised to change default password.

### Resetting router to default settings ###

In case of misconfiguration it is possible to reset router to working default settings (as shipped to you). To do it press and hold reset button on powered on RUTX11 for at least six seconds. Signal strength LEDs indicate elapsed time. After all five LEDs are lit up, reset button can be released. After automatic restart router is ready to be used.

## Docs and links ##
All helpful documents and links in one place:

* [Safety instructions](/docs/assets/pdf/Husarion_Panther_safety_instructions.pdf "Husarion Panther safety instructions") - to avoid malfunction or damage your Panther please read this safety manual before use,
* [Panther schematic block diagram](/docs/assets/pdf/schematic_block_diagram.pdf "Panther schematic block diagram") - basic robot components and connections between them,
* [Overall dimensions](https://files.husarion.com/panther/external_dimentions.pdf "Overall dimensions") - three basic projections of the platform,
* [Teltonika RUTX11 manual](https://wiki.teltonika-networks.com/view/RUTX11_Manual)
* [Teltonika RUTX11 datasheet](https://teltonika-networks.com/downloads/en/rutx11/RUTX11-Datasheet.pdf)


