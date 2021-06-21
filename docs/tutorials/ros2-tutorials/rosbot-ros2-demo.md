---
title: ROS2 Demo for ROSbot 2.0
sidebar_label: 0. ROSbot Demo
id: rosbot-ros2-demo
---

## Intro

The goal of this tutorial is to launch navigation2 demo, which will allow the robot to navigate autonomously and avoid obstacles. The ROSbot will be visualized and controlled through Rviz.

<div style="text-align: center">
<iframe width="784" height="441" src="https://www.youtube.com/embed/fY-Z9OqY0eo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

## Install ROS2 image on ROSbot

### Get a system image

Working on your laptop, visit https://husarion.com/downloads/, find **Ubuntu 20.04 + ROS2 Foxy + Docker + Husarnet client** and download:
- **Tinker Board** version for ROSbot 2.0 
- **UpBoard** version for ROSbot 2.0 PRO

Instructions for system instalation can be found [here](https://husarion.com/manuals/rosbot/#system-reinstallation).

## First ROSbot Configuration

### Connecting to Wi-Fi

In the ROSbot 2.0 set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. To connect with ROSbot via ssh, type in terminal application on the computer: 
    ```bash
    ssh husarion@192.168.0.1
    ```
    a password is also`husarion`

5. To connect to a Wi-Fi network in the terminal type 
    ```bash
    nmcli d wifi connect <WiFi-SSID> password <WiFi-PASSWORD>
    ``` 
6. type `ip a` to find your IP address. Save it for later.

### Flashing STM32 firmware
If this is your first launch of ROS2 Foxy on ROSbot make sure to upgrade STM32 firmware to at least 0.14.3 version (pay attention because version for ROS Noetic is different).

To upgrade firmware launch terminal in the ROSbot's Linux and download the STM32 firmware:

```bash
wget -O /home/husarion/firmware.bin https://files.husarion.com/images/rosbot-2.0-fw-v0.14.3.bin
```

Then flash the STM32 microcontroller:

```bash
bash /home/husarion/flash_firmware.sh
```

More information about flashing the firmware you can find [here](https://husarion.com/manuals/rosbot/#low-level-firmware-installation).

### Remote Desktop (optional)

Remote Desktop is a tool enabling user to connect to a computer in another location and see that computer screen and interact using graphical interface.

By default a VNC server is not running on ROSbot. To start VNC first connect via ssh to robot and then execute following command:

```bash
sudo systemctl start vncserver.service
```

To make use of it you have to ether install [tigervnc](https://tigervnc.org/) or use [docker containers](https://github.com/husarion/rosbot-remote) that we have prepared. Clone project to your computer and build. 

```bash
git clone https://github.com/husarion/rosbot-remote.git
cd rosbot-remote/tiger_vnc
./build_image.sh
./run_image.sh <ip-address-of-rosbot>
```

For more information please take a look at [readme](https://github.com/husarion/rosbot-remote/blob/main/README.md). 

If you are unfamiliar with docker or don't know how to install please use [nvidia guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (to run images it is compulsory to have nvidia-docker2 package installed)

## Launching a demo project

Once you are connected to ROSbot either with ssh or remote desktop launch terminal and type following command:

```bash
source ~/husarion_ws/install/setup.bash
```
and for ROSbot 2.0:
```
ros2 launch rosbot_description navigation_demo.launch.py
```
or for ROSbot 2.0 PRO:
```
ros2 launch rosbot_description navigation_demo_pro.launch.py
```

You can find source code [here](https://github.com/husarion/rosbot_description/blob/foxy/launch/navigation_demo.launch.py) and version for Rosbot PRO is [here](https://github.com/husarion/rosbot_description/blob/foxy/launch/navigation_demo_pro.launch.py) 

## Using a demo

Once ROSbot launched lidar should begin to rotate and logs starts to pop out on terminal. At this point to make your ROSbot move or see how map is being created you need to launch rviz2. We also prepared ready to run docker container with configured rviz2. Instruction is very similar to this with tigervnc client and if you already cloned the repository there is no need to do this again. 

```bash
git clone https://github.com/husarion/rosbot-remote.git
cd rosbot-remote/rviz
./build_image.sh
./run_image.sh
```

If you have ros2 Foxy installed on your system just type `rviz2` to launch rviz.

Add displays you want but for navigation add at least map and select global frame to map.

![image](/docs/assets/img/ros2-tutorials/rviz2.png)

To add destination use green "2D Goal Pose" arrow in the top bar. 

![image](/docs/assets/img/ros2-tutorials/rviz2-nav2.png)

If you want to see rosbot model displayed in rviz2 you will have to select the `rosbot_description/urdf/rosbot.urdf` file in RobotModel section.

![image](/docs/assets/img/ros2-tutorials/rviz2-urdf.png)

**NOTE** Currently dds implementations are heavy on resources so please limit shown displays. 

### Using many ROS2 devices it the same network (Optional)

ROS2 (DDS) provide a concept of multi cast for machines discovery. As a result every machine running ROS2 can see every nodes/topics on all running machines in the same network. 

If you want to limit this behavior export `ROS_DOMAIN_ID` this might be number 0-255 (default is 0). If you want to communicate only two machines add the following line to ~/.bashrc. 

```bash
echo "export ROS_DOMAIN_ID=14" >> ~/.bashrc
```

Other option is to follow our guide for set up [Husarnet with ROS2](https://husarnet.com/docs/tutorial-ros2/). This will enable you to communicate using p2p VPN from any network.  

---

## Using Gazebo simulation

There is also a possibility to run simulated ROSbot in Gazebo. To launch the simulation you will have to install [ROS2 Foxy](https://hub.docker.com/r/rosplanning/navigation2), [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu), and [Navigation2](https://navigation.ros.org/build_instructions/index.html) on your computer. Alternatively use [docker image](https://hub.docker.com/r/rosplanning/navigation2).

When all the prerequisites are met you can clone and build rosbot_description package.

```bash
mkdir -p rosbot_ws/src
cd rosbot_ws/src
git clone https://github.com/husarion/rosbot_description --branch=foxy
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

Once the package is built you can launch the simulation.

```bash
source ~/rosbot_ws/install/setup.bash
ros2 launch rosbot_description navigation_demo_sim.launch.py
```

If Gazebo doesn't start in few seconds try exporting gazebo resource path 
```bash
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
```
and relaunch the simulation in the same terminal.

_by Adam Krawczyk and Kamil Machoń, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
